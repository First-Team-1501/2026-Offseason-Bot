<#
.SYNOPSIS
One pull/analyze cycle of the 1501 continuous tuning system (Task Scheduler runs it every minute).

.DESCRIPTION
- Lock file tuning\watch.lock: a cycle is skipped while another run holds the lock; a lock older than
  15 minutes is treated as stale and replaced.
- Runs pull.py. If it copied or adopted a log, or the active log (.partial) changed, runs analyze.py --new.
- Appends to tuning\watch.log (rolled to watch.log.1 above 1 MB). "Robot unreachable" is logged only when
  the reachability state changes (tuning\watch.state), so the log does not grow by one line per minute.
#>
param(
    [switch]$NoAnalyze,
    [switch]$DryRun,                                                   # pull.py --dry-run, no analysis (tests)
    [string]$TuningDir = 'C:\workspace\Robotics\2026\offseason-bot-wip\tuning'
)

$ErrorActionPreference = 'Continue'
$Tools = $PSScriptRoot
$Tuning = $TuningDir
$LogFile = Join-Path $Tuning 'watch.log'
$LockFile = Join-Path $Tuning 'watch.lock'
$StateFile = Join-Path $Tuning 'watch.state'
$Python = 'C:\Users\jgarlits\AppData\Local\Python\pythoncore-3.14-64\python.exe'
if (Test-Path $Python) { $PyArgs = @('-I') } else { $Python = 'py'; $PyArgs = @('-3.14', '-I') }
New-Item -ItemType Directory -Force -Path $Tuning | Out-Null

function Write-WatchLog([string[]]$Lines) {
    $stamp = (Get-Date).ToString('yyyy-MM-ddTHH:mm:ss')
    try {
        if ((Test-Path $LogFile) -and ((Get-Item $LogFile).Length -gt 1MB)) {
            Move-Item -Force -Path $LogFile -Destination "$LogFile.1"
        }
        $Lines | ForEach-Object { "$stamp $_" } | Add-Content -Path $LogFile -Encoding utf8
    } catch { }
}

# ---- lock (CreateNew is atomic: only one cycle can create it)
if (Test-Path $LockFile) {
    $age = (Get-Date) - (Get-Item $LockFile).LastWriteTime
    if ($age.TotalMinutes -lt 15) { exit 0 }
    Write-WatchLog "watch: stale lock ($([int]$age.TotalMinutes) min) replaced"
    Remove-Item -Force -Path $LockFile -ErrorAction SilentlyContinue
}
try {
    $lockStream = [System.IO.File]::Open($LockFile, [System.IO.FileMode]::CreateNew, [System.IO.FileAccess]::Write)
    $bytes = [System.Text.Encoding]::UTF8.GetBytes("pid $PID $(Get-Date -Format o)")
    $lockStream.Write($bytes, 0, $bytes.Length)
    $lockStream.Close()
} catch {
    exit 0   # another cycle created the lock first
}

try {
    $pullArgs = @((Join-Path $Tools 'pull.py'))
    if ($DryRun) { $pullArgs += '--dry-run'; $NoAnalyze = $true }
    # The active log (.partial) is refreshed at most every 5 minutes: each refresh is an ssh copy on a rio whose CPU is
    # already saturated, possibly while the robot is enabled. Completed logs are pulled every cycle.
    $PartialStamp = Join-Path $Tuning 'watch.partial'
    $partialDue = (-not (Test-Path $PartialStamp)) -or (((Get-Date) - (Get-Item $PartialStamp).LastWriteTime).TotalMinutes -ge 5)
    if ($partialDue) { Set-Content -Path $PartialStamp -Value (Get-Date -Format o) } else { $pullArgs += '--no-partial' }
    $pullOut = & $Python @PyArgs @pullArgs 2>&1 | ForEach-Object { "$_" }
    $pullCode = $LASTEXITCODE
    $result = $pullOut | Where-Object { $_ -like 'RESULT *' } | Select-Object -Last 1
    $prevState = if (Test-Path $StateFile) { (Get-Content -Path $StateFile -Raw).Trim() } else { '' }
    if ($pullCode -eq 2) {
        if ($prevState -ne 'unreachable') { Write-WatchLog 'watch: robot unreachable' }
        Set-Content -Path $StateFile -Value 'unreachable'
        exit 0
    }
    Set-Content -Path $StateFile -Value 'reachable'
    if ($prevState -ne 'reachable') { Write-WatchLog 'watch: robot reachable' }

    $copied = 0; $adopted = 0; $partial = 'none'
    if ($result -match 'copied=(\d+)') { $copied = [int]$Matches[1] }
    if ($result -match 'adopted=(\d+)') { $adopted = [int]$Matches[1] }
    if ($result -match 'partial=(\S+)') { $partial = $Matches[1] }
    $changed = ($copied -gt 0) -or ($adopted -gt 0) -or ($partial -eq 'changed')
    if ($changed -or $pullCode -ne 0 -or $DryRun) {
        Write-WatchLog (@("pull exit $pullCode") + $pullOut)
    }
    if ($changed -and -not $NoAnalyze) {
        $anOut = & $Python @PyArgs (Join-Path $Tools 'analyze.py') '--new' 2>&1 | ForEach-Object { "$_" }
        Write-WatchLog (@("analyze exit $LASTEXITCODE") + $anOut)
    }
} catch {
    Write-WatchLog "watch: ERROR $($_.Exception.Message)"
} finally {
    Remove-Item -Force -Path $LockFile -ErrorAction SilentlyContinue
}
exit 0
