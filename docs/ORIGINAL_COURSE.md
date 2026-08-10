# 원작의 코스·체크포인트·랩 판정

원본 `KartRider.exe`(SHA-256 `812FB0FF…B58C`)와 원본 `track.1s`만 보고 확인한
내용이다. 판정 로직, 게이트 지오메트리의 파일 형식, 리스폰까지 전부 확인했고
구현도 끝났다 — 마지막 절에 구현 위치와 남은 미확인 항목을 적는다.

## 요약

체크포인트는 **도로를 가로지르는 사각형 게이트**다. 카트의 이동 선분이 그
사각형을 뚫으면 통과로 치고, 이동 방향과 게이트 법선의 내적 부호로 정주행과
역주행을 가른다. 게이트들은 forward/backward 링크를 가진 그래프를 이루고,
랩은 "결승선 통과"가 아니라 **누적 진행량이 랩 수와 일치하는 순간** 올라간다.

## 코드 경로

| 주소 | 역할 |
|---|---|
| `0x004240f0` | `track` 오브젝트 property의 `course` 태그를 찾아 그래프 빌드를 트리거, 출발 포즈 계산 |
| `0x00424e00` | `course` DSL(`road`/`plane`/`branch`)을 노드 그래프로 |
| `0x00426470` | 궤적 선분 vs 링크 게이트 → `advance` ±1, 현재 노드 이동 |
| `0x00425fe0` | 게이트 통과 판정과 부호 |
| `0x00434b40` | 선분-삼각형 교차 (Möller–Trumbore, `t∈[0,1]`) |
| `0x00424b30` | per-kart 갱신: `advance` 누적, 랩 카운트, 베스트 랩, 역주행 플래그 |
| `0x00426670` | 현재 노드 안에서의 진행 거리와 현재 포인트 |
| `0x00424530` → `0x004260e0` | 출발 그리드 배치 (레이스 진입 시) |
| `0x00424640` | 리스폰 — 현재 노드로 되돌림 |
| `0x00458000` | 리스폰 트리거: 낙하 또는 `"리셋"` 명령 → 500 ms 뒤 `0x00424640` |
| `0x004ba800` | `ToRoad::Decode` |
| `0x004bae10` → `0x00427bf0` | 이름으로 오브젝트 조회 후 `the::ToRoad`로 캐스트 |

상수: `DAT_00571808` = `0.0`, `_DAT_00571490` = `1.0`, `_DAT_0057180c` = `-2.0`,
`_DAT_005717b8` = `-0.5`, `DAT_005b16f0` = `(0,0,1)`(`0x0056c2f0`에서 초기화),
`DAT_005b16cc` = `(0,0,0)`.

## 에셋

`course`는 `track` 오브젝트의 property 블록에 있는 바이너리 XML 태그다.

```xml
<course>
  <road name="RoadObj01" start="start" end="end"/>
</course>
```

바이너리 XML 태그 하나의 인코딩은 다음과 같다. 전부 UTF-16LE, 길이는 문자 수다.

```
[int len][name][int len][text][int n][ (key,value) × n ][int n][ child × n ]
```

`track` 오브젝트는 클래스 스탬프 `aa 47` + `4c 04 53 19`로 찾는다. 그 property
루트 태그의 이름은 `property`이고, `camera`/`fog`/`course`/`courseReverse`/`sun`
을 자식으로 갖는다. `courseReverse`는 역주행 모드용이라 `0x004240f0`은 읽지
않는다. `branch` 안의 `course`는 그 분기의 대안 코스다.

`desert_I02`의 track.1s에는 `track` 오브젝트가 **두 개** 있고, 두 번째 것의
course는 그 파일에 존재하지 않는 `RoadObj04/08/09`를 가리킨다. 추출기는 파일
안의 ToRoad로 전부 해소되는 쪽을 쓴다.

`name`이 가리키는 것은 컨테이너 꼬리 섹션의 **`the::ToRoad` 오브젝트**이고,
씬 트리의 `RoadObj01@sN` 렌더 메쉬가 아니다. 둘은 별개다.

### `ToRoad` 온디스크 형식 (13개 트랙 전부에서 확인)

```
[aa 47][stamp 49 02 de 07][uint16 obj index]
[int len]"RoadObj01"            UTF-16
byte   property 유무            != 0 이면 태그 하나가 이어짐
byte   ?
int    원소 개수                예: 49
원소마다:
    [int len][name]             빈 문자열이 대부분. start/end/final/분기점만 이름을 가짐
    int    vertexCount          4 또는 12
    vec3   vertices[vertexCount]
    int    gateTriangleCount    항상 2      ← face 0 / face 1 = 게이트
    uint16 gateTriangles[][3]
    [int len][extra]            빈 문자열. ice_R01에 "warpnext" 하나뿐
    int    wallTriangleCount    0, 4, 6, … ← 측벽. 판정에는 안 쓰임
    uint16 wallTriangles[][3]
    int    recordCount
    record records[recordCount] stride 0x24: vec3 position, vec3 direction, vec3 up
```

인메모리 레이아웃이 필드 순서 그대로 맞는다 — name `+0x00`, vertices `+0x04`
(`0x00426d00`, stride `0xc`), gate triangles `+0x10`(`0x00426d20`, stride `6`),
extra 문자열 `+0x1c`, walls `+0x20`, records `+0x2c`(`0x00426d60`, stride
`0x24`), 전체 `0x38`(`0x00426d80`).

**이름은 원소 앞에 붙는다.** 뒤에 붙는 것으로 읽으면 원소 하나만큼 어긋나는데,
`forest_I01`이 그걸 갈라준다: `start`라고 이름 붙은 원소의 게이트 평면이
y = 481.1747이고, 씬의 출발선 메쉬 `RoadObj01@s4`의 near edge가 정확히 같은
481.17474다. 13개 트랙 중 12개에서 `start` 원소의 게이트가 출발선 스트라이프
사각형 안에 떨어진다 (`ice_I02`만 2.5 유닛 앞에 있다).

원소 0의 값이 정체를 그대로 보여준다.

```
v0 (784.61, 416.65, 25.89)   v1 (732.61, 416.65, 25.89)
v2 (784.61, 416.65, 45.89)   v3 (732.61, 416.65, 45.89)
gateTriangles = (0,1,3), (0,3,2)
record[0] = pos (758.61, 416.65, 26.90)  dir (0,1,0)  up (0,0,1)
```

y=416.65 평면에 선 52×20 직사각형과 그것을 이루는 두 삼각형, 그리고 그 평면
위의 중심선 점이다. 세계 좌표는 X/Y가 지면, **Z가 높이**다.

## 그래프 빌드 (`0x00424e00`, road 케이스)

```
name    → 씬에서 오브젝트 조회, 그 +0x14가 원소 리스트
start   → 이름이 일치하는 원소의 인덱스          기본 0
end     → 〃                                      기본 N-1
final   → 〃                                      기본 N  (= 어느 원소도 final이 아님)
reverse → 순회 방향과 법선 부호. "1"과 "true" 둘 다 쓰인다

cur = 새 노드(id = 시작 id);  prev = {}
반복:
    next = reverse ? (cur==0 ? N : cur) - 1
                   : (cur==N-1 ? 0 : cur+1)
```

원소마다 0x5c바이트 **게이트**를 만든다.

| 오프셋 | 내용 |
|---|---|
| `+0x04` | face0 의 `vertex[ index[0] ]` |
| `+0x10` | face0 의 `vertex[ index[reverse?2:1] ]` |
| `+0x1c` | face0 의 `vertex[ index[2-reverse] ]` |
| `+0x28` | face1 의 `vertex[ index[0] ]` |
| `+0x34` | face1 의 `vertex[ index[reverse?2:1] ]` |
| `+0x40` | face1 의 `vertex[ index[2-reverse] ]` |
| `+0x4c` | 레코드 0의 direction × (reverse ? −1 : +1) |
| `+0x58` | 이 원소가 `final` 이면 1 |

**노드**(0x30바이트)는 `+0x04` id, `+0x08` backward 링크 리스트, `+0x14`
forward 링크 리스트, `+0x20` 중심선 포인트(24바이트 = pos + dir), `+0x2c` 그
폴리라인의 길이 합. 링크는 8바이트 `{게이트, 노드}` 쌍이다.

원소 하나를 처리할 때:

```
게이트 g를 만들고
if (prev 비었음)  cur.forward  += {g, null}      ← 링 닫기나 분기 병합이 나중에 채움
else for (p in prev) { p.backward += {g, cur};  cur.forward += {g, p}; }

// 중심선. reverse면 원소 next의 레코드를 역순으로, direction을 뒤집어서 넣는다
if (!reverse) for (r in elem[cur].records)              cur.points += {r.pos,  r.dir}
else          for (r = elem[next].records 역순)          cur.points += {r.pos, -r.dir}

cur.length = Σ |points[k+1].pos − points[k].pos|
노드리스트.append(cur);  prev = {cur};  cur = 새 노드(id = 직전 id + 1)
if (원소 == end) break
```

**필드 이름이 뒤집혀 있다.** `p.backward`에 들어가는 노드는 p 다음 노드고,
`cur.forward`에 들어가는 노드는 cur 이전 노드다. `0x00426470`이 backward를
양(+) 통과로, forward를 음(−) 통과로 검사하는 것과 일관된다.

노드 i는 게이트 i **뒤에 이어지는 구간**이다. 그래서 첫 노드의 첫 포인트가
출발선 위에 있고, 카트는 마지막 노드에서 출발한다.

`0x00424e00`은 항상 아직 발행하지 않은 노드를 하나 들고 있어서 구간마다,
그리고 분기 대안마다 잉여 노드가 하나씩 남는다. 원본의 노드 벡터에는 발행된
것만 들어간다.

### 링 닫기 (`param_6 != 0`, 최상위 코스만)

```
link = 첫 노드.forward[0]        // {g0, null}
link.node = 마지막 노드;  첫 노드.forward[0] = link
마지막 노드.backward += {g0, 첫 노드}
```

### 분기 (`branch`)

자식 `course` 하나하나를 독립 코스로 빌드한 뒤 분기 앞뒤 노드에 이어 붙인다.

```
for (대안 sub in branch의 자식들) {
    sub를 param_6=0 으로 빌드
    entryGate = sub.first.forward[0].gate;   sub.first.forward 비움
    for (p in prev) { sub.first.forward += {entryGate, p};  p.backward += {entryGate, sub.first} }
    for (l in sub.last.forward) l.node.backward[0].node = cur     // 합류점을 cur로 재조준
    cur.forward += {sub.last.forward[0].gate, sub.last.forward[0].node}
    노드리스트 += sub의 마지막 노드를 뺀 전부
    if (첫 대안) cur.points = sub.last.points; cur.length 재계산
}
cur.id = max(대안들의 마지막 id) − 1;  발행;  prev = {cur};  cur = 새 노드
```

즉 분기의 각 대안은 **같은 합류 노드**로 들어오고, 그 합류 노드의 중심선은
첫 번째 대안의 마지막 구간에서 가져온다.

13개 트랙에는 `road`와 `branch`만 쓰인다. `0x00424e00`이 받는 `plane`은
하나도 없어서 구현하지 않았다.

## 통과 판정 (`0x00425fe0`)

인자는 게이트, 선분 시작, 선분 끝이다.

```c
dir = segEnd - segStart;
hit = 선분_삼각형_교차(gate+0x04, segStart, dir);      // face 0
if (!hit) hit = 선분_삼각형_교차(gate+0x28, segStart, dir); // face 1
if (!hit) return 0;
return dot(dir, gate->[0x4c]) < 0 ? -1 : +1;
```

반경도 여유폭도 없다. 임계값은 `0` 하나뿐이다. `0x00434b40`은 교차 파라미터
`t`, `u`, `v`를 전부 `[0,1]`로 자르는 선분 판정이라 게이트 앞에서 멈추면 통과가
아니고, 게이트 옆으로 돌아가면 아예 걸리지 않는다.

## 진행과 랩 (`0x00426470`, `0x00424b30`)

```c
// 0x00426470 — 카트의 최근 궤적(kart+0x318, vec3 배열) 선분마다
if (count(trail) < 2) return 0;
for (seg in trail) {
    for (link in cur->[0x08])                  // backward = 다음 노드들
        if (통과(link.gate, seg) > 0) {
            if (link.node == course->[0x58]) advance = +1;              // 첫 노드로 진입
            else if (link.gate->[0x58] && advanceOf(kart) == course->[0x60])
                advance = +1;                                            // final 게이트, 마지막 랩
            cur = link.node; return 1;
        }
    for (link in cur->[0x14])                  // forward = 이전 노드들
        if (통과(link.gate, seg) < 0) {
            if (cur == course->[0x58]) advance = -1;
            cur = link.node; return 1;
        }
}

// 0x00424b30 — per-kart
if (advance == 1 && record[1] == record[7]) {
    if (record[7] == 0) { record[6] = time; record[7]++; }
    else {
        lap = time - record[6];
        record[9] = record[9] ? min(record[9], lap) : lap;   // 베스트 랩
        record[6] = time; record[7]++; record[8] = 1;
    }
}
record[1] += advance;
record[2] = cur.id;
0x00426670(kart, record);        // record[3] 진행 거리, record[4] 현재 포인트
record[10] = 역주행 판정;
```

`advance`는 **시작 노드를 넘을 때만** ±1이 된다. 다른 노드는 현재 노드만
옮긴다. 그래서 지름길로 게이트를 건너뛰면 누적 진행량이 랩 수와 어긋나 랩이
올라가지 않고, 역주행으로 시작 노드를 넘으면 −1로 깎인다.

`course->[0x58]`은 노드 리스트의 **첫 노드**, `course->[0x5c]`는 **마지막
노드**, `course->[0x60]`은 랩 수(`0x004247e0`이 세팅)다.

### 노드 안에서의 진행 거리 (`0x00426670`)

```c
p = node.points.begin();
while (p != end && dot(kartPos - p.pos, p.dir) >= 0) ++p;
record[3] = 0;
if (p != begin) {
    for (c = begin; c + 1 != p; ++c) record[3] += |c[1].pos - c[0].pos|;
    record[3] += dot(kartPos - c.pos, c.dir);       // c = p - 1
}
record[4] = c;
if (record[3] > node.length) record[3] = node.length;
```

### 역주행 (`0x00424b30` 꼬리, `record[10]`)

```c
ok = true;
for (ahead in node->[0x08]) for (behind in node->[0x14]) {
    travel = normalize(ahead.node.points[0].pos - behind.node.points[0].pos);
    if (dot(travel, kartForward)          > -0.5) { ok = false; break; }
    if (dot(travel, normalize(kartVel))   > -0.5) { ok = false; break; }
}
record[10] = ok;
```

`kartForward`는 `0x00426aa0`이 반환하는 값 — 카트 행렬 `+0x38`의 **열 1을
음수화**한 것이고, 이 저장소의 `orientation_axes()`가 forward라고 부르는 축과
같다. `kartVel`은 `kart+0x5c`이며, `0x00452bb0`이 그 길이를 속도계에 넣는 것과
`0x00428c40`이 순간이동 시 0으로 지우는 것으로 확인했다.

즉 카트가 **바라보는 방향과 움직이는 방향이 둘 다** 코스를 거스를 때만 참이다.
한쪽만 어긋나면(스핀 등) 꺼진다. `-0.5`는 120도 원뿔이라 급코너에서는 역주행
중에도 잠깐 꺼진다.

## 출발 포즈와 리스폰

### 코스가 정하는 출발 포즈 (`0x004240f0` 꼬리)

```c
p    = 첫 노드.points[0];
col1 = -p.dir;
col0 = normalize(cross(col1, (0,0,1)));
col2 = normalize(cross(col0, col1));
course->[0x34] = {col0, col1, col2};            // 열 우선
course->[0x28] = p.pos + 0.5 * col1;            // 게이트 0.5 뒤
```

카트의 forward는 기저 열 1의 음수이므로, 이 포즈는 코스 진행 방향을 향한다.
**출발선 메쉬만으로는 알 수 없던 주행 방향의 부호가 여기서 확정된다.**

### 출발 그리드 (`0x00424530` → `0x004260e0`)

```c
offset = (i % 2 == 0) ? i : ((i>>1) + 1) * -2.0;      // 0, -2, +2, -4, +4, …
pos    = course->[0x28] + offset * column0(course->[0x34]);
rot    = course->[0x34];
if (레이캐스트(pos + (0,0,10), (0,0,-100), &hit)) pos = hit.point;
SetTransform(kart, pos, rot);
record = { 마지막 노드, advance 0, 그 노드의 id };
```

레이스 진입(`0x004511b0`, `0x00455c80`)에서만 불린다.

### 리스폰 (`0x00424640`)

```c
node = record[0];                     // 카트가 지금 들어 있는 노드
p    = node.points[0];                // 가장 가까운 포인트가 아니라 0번
col1 = -p.dir;
col0 = cross(col1, (0,0,1));          // 여기서는 정규화하지 않는다
col2 = cross(col0, col1);
SetTransform(kart, p.pos + 0.5*col1, {col0, col1, col2});   // 속도도 0이 된다
```

`0x00458000`이 이걸 부른다. 카트가 `_DAT_0057330c` 아래로 떨어지거나 카트에
`"리셋"` 명령이 들어오면 같은 가상 함수(`this+0x24`)가 `this+0xd0`에 시각을
찍고, 그 뒤 `t0+500 ≤ t < t0+1000` 구간에서 매 프레임 `0x00424640`을 부른다.
`t0+1000` 이후에는 `0x0043ff00(kart, 0)`, `t0+3000`에는 `0x00459510(kart, 1)`을
부르는데 이 둘은 아직 해석하지 않았다.

**출발 그리드와 리스폰은 다른 동작이다.** 그리드는 코스 전체의 출발 포즈로
가고 지면에 스냅하지만, 리스폰은 현재 노드로만 되돌리고 레이캐스트를 하지
않는다.

## 구현

| 파일 | 내용 |
|---|---|
| `DeveloperTools/AssetPipeline/derive_course_gates.py` | track.1s에서 course 태그와 ToRoad 게이트를 뽑아 `Scripts/Runtime/Gameplay/kart_course_data.c` 생성 |
| `Scripts/Runtime/Gameplay/kart_course_data.c` | 생성물. 13개 트랙, 23개 ToRoad 오브젝트 |
| `Scripts/Runtime/Gameplay/kart_course.h`, `Scripts/Runtime/Gameplay/kart_course.c` | 그래프 빌드, 통과 판정, 진행·랩·역주행, 출발/리스폰 포즈 |
| `Scripts/Tests/test_course_progress.c` | 13개 트랙 전부에서 중심선을 주행시켜 랩 카운트를 검증 |
| `Scripts/Tests/test_track_scene_assets.c` | 실제 씬 위에서 출발 그리드 스냅과 첫 게이트 통과를 검증 |

생성 데이터의 좌표는 `kart_track_scene_world_vertex`가 씬 정점에 적용하는
것과 같은 변환을 이미 거친 월드 좌표다 (X 미러 포함). 방향 벡터에는 그 미러의
선형 부분만 적용한다.

## 미확인

- **궤적 버퍼의 길이.** `0x00426470`은 `kart+0x318`의 vec3 배열을 훑고 원소가
  2개 이상일 것만 요구한다. 몇 개를 유지하는지는 확인하지 않았다. 구현은 한
  시뮬레이션 스텝당 선분 하나(직전 위치 → 현재 위치)를 넣는다.
- **랩 수의 출처.** `0x004247e0`이 `course->[0x60]`에 넣는 값은 씬 오브젝트의
  `+0x28`/`+0xd8`에서 온다. 트랙 에셋에는 없다. 구현은 3랩을 쓴다.
- `0x0043ff00`, `0x00459510`, `_DAT_0057330c` — 리스폰 시퀀스의 500 ms 이후
  단계.
- `0x00424e00`의 `plane` 케이스. 13개 트랙 어디에도 없다.
- `ice_R01`의 `RoadObj16`은 `end="lifein"`을 가리키는데 그런 이름의 원소가
  없다. `0x00424e00`의 기본값(N−1)으로 떨어진다.
