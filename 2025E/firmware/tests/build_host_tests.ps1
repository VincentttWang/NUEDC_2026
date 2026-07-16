# build_host_tests.ps1 — 主机侧编译并运行 core 层纯计算单元测试。
#
# 不进 Keil/CCS 工程, 不碰 MCU 头; 只编译 core/** (+ auto_aim 用 tests/stubs 打桩)。
# 用法:  pwsh -File tests/build_host_tests.ps1
# 退出码: 全部通过=0, 任一失败/编译错=1。

$ErrorActionPreference = "Stop"
$fw = Split-Path -Parent $PSScriptRoot        # firmware/
$out = Join-Path $env:TEMP "nuedc_hosttest"
New-Item -ItemType Directory -Force -Path $out | Out-Null

# 选择编译器: clang 优先, 退回 gcc。
$cc = $null
foreach ($cand in @("clang", "gcc")) {
    if (Get-Command $cand -ErrorAction SilentlyContinue) { $cc = $cand; break }
}
if (-not $cc) {
    Write-Error "未找到 clang 或 gcc, 无法编译主机单测。请安装其一并加入 PATH。"
    exit 1
}
Write-Host "compiler: $cc" -ForegroundColor Cyan

$cflags = @("-std=c11", "-Wall", "-Wextra", "-Wno-unused-parameter")
# libm 仅 gcc/mingw 需要; MSVC 后端的 clang 数学函数在 CRT 里, 加 -lm 会找 m.lib 失败。
$libs = if ($cc -eq "gcc") { @("-lm") } else { @() }

# 每个测试: 名称 -> (源文件列表, include 路径列表)
$tests = @(
    @{ name = "test_aim_solver";
       src  = @("core/kinematics/kinematics.c", "core/aim_solver/aim_solver.c",
                "tests/test_aim_solver.c");
       inc  = @("core") },
    @{ name = "test_localization";
       src  = @("core/kinematics/kinematics.c", "core/localization/localization.c",
                "tests/test_localization.c");
       inc  = @("core") },
    @{ name = "test_aim_fusion";
       src  = @("core/kinematics/kinematics.c", "core/aim_solver/aim_solver.c",
                "core/aim_fusion/aim_fusion.c", "tests/test_aim_fusion.c");
       inc  = @("core") },
    @{ name = "test_aim_localization";
       src  = @("core/kinematics/kinematics.c", "core/aim_solver/aim_solver.c",
                "core/localization/localization.c", "tests/test_aim_localization.c");
       inc  = @("core") },
    @{ name = "test_auto_aim";
       src  = @("core/kinematics/kinematics.c", "core/aim_solver/aim_solver.c",
                "core/aim_fusion/aim_fusion.c", "core/localization/localization.c",
                "middleware/auto_aim/auto_aim.c", "tests/test_auto_aim.c");
       inc  = @("tests/stubs", ".", "core", "bsp/common") },
    @{ name = "test_gimbal_tracking";
       src  = @("core/kinematics/kinematics.c", "core/geometry/geometry.c",
                "core/pid/pid.c", "middleware/gimbal_tracking/gimbal_tracking.c",
                "tests/test_gimbal_tracking.c");
       inc  = @("tests/stubs", ".", "core", "bsp/common") }
)

$fail = 0
foreach ($t in $tests) {
    $exe = Join-Path $out ($t.name + ".exe")
    $srcAbs = $t.src | ForEach-Object { Join-Path $fw $_ }
    $incAbs = $t.inc | ForEach-Object { "-I" + (Join-Path $fw $_) }
    $args = @($cflags) + $incAbs + $srcAbs + @("-o", $exe) + $libs

    Write-Host "`n=== build $($t.name) ===" -ForegroundColor Yellow
    & $cc @args
    if ($LASTEXITCODE -ne 0) { Write-Host "[BUILD FAIL] $($t.name)" -ForegroundColor Red; $fail = 1; continue }

    & $exe
    if ($LASTEXITCODE -ne 0) { Write-Host "[TEST FAIL] $($t.name)" -ForegroundColor Red; $fail = 1 }
    else { Write-Host "[OK] $($t.name)" -ForegroundColor Green }
}

Write-Host ""
if ($fail -eq 0) { Write-Host "ALL HOST TESTS PASSED" -ForegroundColor Green }
else { Write-Host "HOST TESTS FAILED" -ForegroundColor Red }
exit $fail
