Set-Location "C:\Users\Lenovo\Downloads\main_project"
$armclang = "C:\Users\Lenovo\AppData\Local\Keil_v5\ARM\ARMCLANG\Bin\armclang.exe"
$armlink   = "C:\Users\Lenovo\AppData\Local\Keil_v5\ARM\ARMCLANG\Bin\armlink.exe"
$fromelf   = "C:\Users\Lenovo\AppData\Local\Keil_v5\ARM\ARMCLANG\Bin\fromelf.exe"
$uv4       = "C:\Users\Lenovo\AppData\Local\Keil_v5\UV4\UV4.exe"

$cflags = @(
    "--target=arm-arm-none-eabi",
    "-mcpu=cortex-m0",
    "-c",
    "-O1",
    "-D__MICROLIB",
    "-fshort-enums",
    "-fshort-wchar",
    "-IC:\Users\Lenovo\AppData\Local\Arm\Packs\SONiX\SN32F4_DFP\1.1.2\Device\Include",
    "-IC:\Users\Lenovo\AppData\Local\Arm\Packs\ARM\CMSIS\6.3.0\CMSIS\Core\Include",
    "-I.",
    "-IRTE\_Target_1",
    "-IRTE\Device\SN32F407F"
)

function Check-BuildError {
    if ($LASTEXITCODE -ne 0) {
        Write-Error "Build step failed with exit code $LASTEXITCODE"
        exit $LASTEXITCODE
    }
}

Write-Host "Compiling main.c..."
& $armclang @cflags "main.c" -o ".\Objects\main.o"
Check-BuildError

Write-Host "Compiling debug_uart.c..."
& $armclang @cflags "debug_uart.c" -o ".\Objects\debug_uart.o"
Check-BuildError

Write-Host "Compiling sim4g.c..."
& $armclang @cflags "sim4g.c" -o ".\Objects\sim4g.o"
Check-BuildError

Write-Host "Compiling gps.c..."
& $armclang @cflags "gps.c" -o ".\Objects\gps.o"
Check-BuildError

Write-Host "Compiling tinyml_infer.c..."
& $armclang @cflags "tinyml_infer.c" -o ".\Objects\tinyml_infer.o"
Check-BuildError

Write-Host "Compiling system_SN32F400.c..."
& $armclang @cflags ".\RTE\Device\SN32F407F\system_SN32F400.c" -o ".\Objects\system_sn32f400.o"
Check-BuildError

Write-Host "Linking Objects..."
& $armlink --via ".\Objects\MCU_MIENTRUNG.lnp"
Check-BuildError

Write-Host "Generating HEX..."
& $fromelf --i32combined --output=".\Objects\MCU_MIENTRUNG.hex" ".\Objects\MCU_MIENTRUNG.axf"
Check-BuildError

Write-Host "Flashing into MCU via Keil SN-Link..."
Remove-Item ".\flash.log" -ErrorAction SilentlyContinue
Start-Process -FilePath $uv4 -ArgumentList "-f", ".\MCU_MIENTRUNG.uvprojx", "-j0", "-o", ".\flash.log" -Wait
Get-Content ".\flash.log"
Write-Host "Build and Flash Complete!"
