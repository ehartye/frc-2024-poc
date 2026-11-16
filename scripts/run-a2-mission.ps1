param([Parameter(Mandatory=$true)][string]$JavaHome,[Parameter(Mandatory=$true)][string]$GradleHome,[string]$OutputDirectory)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
if(-not $OutputDirectory){$OutputDirectory=Join-Path $root ('artifacts/mission-'+[guid]::NewGuid().ToString())}
python (Join-Path $PSScriptRoot 'run-a2-mission.py') --java-home $JavaHome --gradle-home $GradleHome --output $OutputDirectory
if($LASTEXITCODE -ne 0){throw 'Mission verification failed; pending diagnostics retained.'}
Write-Output "Mission traces and receipt: $OutputDirectory"
