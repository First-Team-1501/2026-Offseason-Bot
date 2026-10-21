<#
.SYNOPSIS
Registers the Windows scheduled task "1501 Robot Log Pull" that runs watch.ps1 every minute while the user is
logged on (hidden console, no elevation).

.DESCRIPTION
Run once, manually, in a normal (non-elevated) PowerShell:
    powershell -NoProfile -ExecutionPolicy Bypass -File register-watch.ps1
Remove with:
    Unregister-ScheduledTask -TaskName '1501 Robot Log Pull' -Confirm:$false
conhost.exe --headless keeps the console window hidden (a plain -WindowStyle Hidden still flashes a window each
minute). If conhost --headless is unavailable, replace the action with:
    powershell.exe -NoProfile -NonInteractive -WindowStyle Hidden -ExecutionPolicy Bypass -File <watch.ps1>
#>
$ErrorActionPreference = 'Stop'
$TaskName = '1501 Robot Log Pull'
$Watch = Join-Path $PSScriptRoot 'watch.ps1'
if (-not (Test-Path $Watch)) { throw "watch.ps1 not found next to this script: $Watch" }

$argument = "--headless powershell.exe -NoProfile -NonInteractive -WindowStyle Hidden -ExecutionPolicy Bypass -File `"$Watch`""
$action = New-ScheduledTaskAction -Execute 'conhost.exe' -Argument $argument -WorkingDirectory $PSScriptRoot
# Repeats every minute with no end; LogonType Interactive runs it only while the user is logged on.
$trigger = New-ScheduledTaskTrigger -Once -At (Get-Date).AddMinutes(1) -RepetitionInterval (New-TimeSpan -Minutes 1)
$user = [System.Security.Principal.WindowsIdentity]::GetCurrent().Name
$principal = New-ScheduledTaskPrincipal -UserId $user -LogonType Interactive -RunLevel Limited
$settings = New-ScheduledTaskSettingsSet -MultipleInstances IgnoreNew -ExecutionTimeLimit (New-TimeSpan -Minutes 30) `
    -StartWhenAvailable -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -Hidden
Register-ScheduledTask -TaskName $TaskName -Action $action -Trigger $trigger -Principal $principal -Settings $settings `
    -Description 'FRC 1501: pull roboRIO logs when reachable and analyze new logs (tools\tuning\watch.ps1).' -Force | Out-Null
Write-Output "Registered '$TaskName' for ${user}: every 1 minute, runs $Watch"
