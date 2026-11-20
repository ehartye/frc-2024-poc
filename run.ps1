param([Parameter(Mandatory=$true)][string]$JavaHome,[Parameter(Mandatory=$true)][string]$GradleHome,[string]$OutputDirectory)
$ErrorActionPreference='Stop'
$argsList=@((Join-Path $PSScriptRoot 'experiments/a2-campaign/scripts/run.py'),'--java-home',$JavaHome,'--gradle-home',$GradleHome)
if($OutputDirectory){$argsList+=@('--output',$OutputDirectory)}
python @argsList
if($LASTEXITCODE -ne 0){throw 'POC execution failed; diagnostics retained.'}
