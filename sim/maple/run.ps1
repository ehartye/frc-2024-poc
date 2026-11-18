param([Parameter(Mandatory=$true)][string]$MapleSource)
$ErrorActionPreference='Stop'
python "$PSScriptRoot\verify_run.py" --maple-source $MapleSource
if ($LASTEXITCODE -ne 0) { throw 'Headless Maple verification failed' }
