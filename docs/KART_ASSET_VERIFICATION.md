# 카트 프리셋 검증

`src/kart_demo_data.c`의 26개 카트 상수를 원본 데모 에셋과 대조한 기록이다.
재실행은 `python scripts/derive_kart_constants.py`.

## 출처

2004년 데모 설치본이다. EXE는 저장소의 `input/KartRider.exe`와 동일하다.

- 경로: `C:\Program Files (x86)\Nexon\KartRider Demo`
- `KartRider.exe` SHA-256 `812FB0FFD032A05C1F89478EA809B781E22EBEF12320D0A2BCE3745CFBFFB58C`
- `Data\kart.rho` SHA-256 `596447E7BCD899BED99D2D474EA18D0EAA1CCA21ACE65FF12A03A3B25D2670D0`, 134 엔트리

`kart.rho`는 카트마다 `model.1s`, `parameter.xml`, `0.png`, `1.png`,
`shadow.png`를 담고 `kartlist.xml`과 `common/`이 따로 있다. `kartlist.xml`에는
`burst3` 한 항목만 들어 있어, 데모가 `burst3`로 시작하는 것과 일치한다.

이 설치본에는 `track_village_C101.rho`, `track_village_C102.rho`도 존재한다.
현대 설치본에 없어서 제거했던 두 트랙이며, 여기서는 재추출이 가능하다.

## 결과

| 항목 | 대조 대상 | 결과 |
|---|---|---|
| Dynamics 16개 × 26카트 | `parameter.xml` | **416/416 일치** |
| 치수 3개 × 26카트 | `model.1s` 본체 메쉬 | **78/78 일치** |
| 3D 형상 | — | **불일치. 메쉬가 아니라 손으로 만든 상자** |

### Dynamics

`parameter.xml`은 5종뿐이고 9/5/5/5/2로 나뉜다. 소스의 `PRACTICE`/`STANDARD`/
`MARATHON`/`SABER`/`SOLID` 매크로 배정과 정확히 같다.

`saber*`, `solid*`의 `parameter.xml`에는 `CornerDrawFactor` 속성이 아예 없고,
소스는 두 매크로 모두 `0.0f`다. 속성 부재 시 0 폴백과 일치한다.

### 치수

각 `model.1s`는 본체 메쉬(index 0) 뒤에 바퀴 4개와 작은 부품이 붙는다.
상수는 **본체 메쉬만**으로 계산된다.

```text
half_width   = (max_x - min_x) / 2
half_length  = (max_y - min_y) / 2
model_height = max_z - min(0, min_z)
```

전체 모델 AABB로는 재현되지 않는다. 뒷바퀴가 차체보다 옆으로 더 튀어나오기
때문이다. `burst3` 기준 본체는 x ±0.8080695인데 뒷바퀴까지 포함하면 ±0.8552700이다.

`model_height`의 0 클램프는 `saber1`에서만 의미가 있다. 26개 중 유일하게 본체
최저점이 z 0보다 아래(`-0.0019963`)이며, 그래서 이 카트만 `max_z`가 아니라
`max_z - min_z = 0.8148401`이 된다.

## 3D 형상은 에셋 기반이 아니다

`demo/kart_3d_win32.c`의 `draw_kart`는 메쉬를 쓰지 않는다. `half_width`,
`half_length`, `model_height`로 8정점 상자를 만들고 지붕을 임의 계수로 좁힌다.

```c
const float roof_width  = half_width  * 0.8f;
const float roof_length = half_length * 0.75f;
```

`0.8`과 `0.75`는 어떤 원본 데이터에도 없다. 치수는 검증됐지만 **형상은 원작과
다르다.** `model.1s`는 이제 `export-kart`로 KTRK 내보내기가 되므로, 트랙과 같은
방식으로 실제 메쉬를 넣는 것은 남은 작업이다. `burst3` 기준 6개 메쉬,
284정점, 442삼각형이다.

## 도구 변경

- `rho-safe-index`: `.xml`/`.txt`/`.ini`를 `config` 종류로 추출 대상에 추가.
  이전에는 `other`로 분류돼 `parameter.xml`이 빠졌다.
- `track-mesh-exporter`: `export-kart` 동사 추가. 루트가 `TrackContainer`가
  아니라 `Relement`이고, 지오메트리가 `ReTriList`/`ReTriStrip`이 아니라
  `ReToonRigid`(정점·법선·텍스처좌표 배열 + 면당 인덱스)라서 둘 다 처리한다.
