param(
    [Parameter(Mandatory = $true)]
    [string]$Executable,
    [Parameter(Mandatory = $true)]
    [string]$Output,
    [switch]$DemoMotion
)

Add-Type -AssemblyName System.Drawing
Add-Type @'
using System;
using System.Runtime.InteropServices;
public static class ScreenshotWindowApi {
    [StructLayout(LayoutKind.Sequential)]
    public struct RECT { public int Left, Top, Right, Bottom; }
    [DllImport("user32.dll")]
    public static extern bool GetWindowRect(IntPtr hWnd, out RECT rect);
    [DllImport("user32.dll")]
    public static extern bool SetForegroundWindow(IntPtr hWnd);
    [DllImport("user32.dll")]
    public static extern bool ShowWindow(IntPtr hWnd, int command);
    [DllImport("user32.dll")]
    public static extern void keybd_event(byte virtualKey, byte scanCode, uint flags, UIntPtr extraInfo);
}
'@

$resolvedExecutable = (Resolve-Path -LiteralPath $Executable).Path
$outputDirectory = Split-Path -Parent $Output
if (-not [IO.Path]::IsPathRooted($Output)) {
    $Output = Join-Path (Get-Location) $Output
    $outputDirectory = Split-Path -Parent $Output
}
[IO.Directory]::CreateDirectory($outputDirectory) | Out-Null

$process = Start-Process -FilePath $resolvedExecutable -PassThru
try {
    $deadline = [DateTime]::UtcNow.AddSeconds(10)
    do {
        Start-Sleep -Milliseconds 100
        $process.Refresh()
    } while ($process.MainWindowHandle -eq 0 -and [DateTime]::UtcNow -lt $deadline)
    if ($process.MainWindowHandle -eq 0) {
        throw "The simulator did not create a window."
    }

    [ScreenshotWindowApi]::ShowWindow($process.MainWindowHandle, 9) | Out-Null
    [ScreenshotWindowApi]::SetForegroundWindow($process.MainWindowHandle) | Out-Null
    Start-Sleep -Milliseconds 1200

    if ($DemoMotion) {
        $keyUp = 0x0002
        [ScreenshotWindowApi]::keybd_event(0x26, 0, 0, [UIntPtr]::Zero)
        [ScreenshotWindowApi]::keybd_event(0x27, 0, 0, [UIntPtr]::Zero)
        [ScreenshotWindowApi]::keybd_event(0x10, 0, 0, [UIntPtr]::Zero)
        Start-Sleep -Milliseconds 1800
        [ScreenshotWindowApi]::keybd_event(0x10, 0, $keyUp, [UIntPtr]::Zero)
        [ScreenshotWindowApi]::keybd_event(0x27, 0, $keyUp, [UIntPtr]::Zero)
        [ScreenshotWindowApi]::keybd_event(0x11, 0, 0, [UIntPtr]::Zero)
        Start-Sleep -Milliseconds 900
        [ScreenshotWindowApi]::keybd_event(0x11, 0, $keyUp, [UIntPtr]::Zero)
        [ScreenshotWindowApi]::keybd_event(0x26, 0, $keyUp, [UIntPtr]::Zero)
        Start-Sleep -Milliseconds 100
    }

    $rect = New-Object ScreenshotWindowApi+RECT
    if (-not [ScreenshotWindowApi]::GetWindowRect($process.MainWindowHandle, [ref]$rect)) {
        throw "Could not read the simulator window rectangle."
    }
    $width = $rect.Right - $rect.Left
    $height = $rect.Bottom - $rect.Top
    $bitmap = New-Object Drawing.Bitmap $width, $height
    $graphics = [Drawing.Graphics]::FromImage($bitmap)
    try {
        $graphics.CopyFromScreen($rect.Left, $rect.Top, 0, 0, $bitmap.Size)
        $bitmap.Save($Output, [Drawing.Imaging.ImageFormat]::Png)
    } finally {
        $graphics.Dispose()
        $bitmap.Dispose()
    }
    Write-Output $Output
} finally {
    foreach ($virtualKey in 0x10, 0x11, 0x26, 0x27) {
        [ScreenshotWindowApi]::keybd_event($virtualKey, 0, 0x0002, [UIntPtr]::Zero)
    }
    if (-not $process.HasExited) {
        $process.CloseMainWindow() | Out-Null
        $process.WaitForExit(3000) | Out-Null
    }
}
