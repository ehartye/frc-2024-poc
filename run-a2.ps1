param([Parameter(Mandatory=$true)][string]$JavaHome,[Parameter(Mandatory=$true)][string]$GradleHome,[string]$OutputDirectory)
$ErrorActionPreference='Stop'
$argsList=@((Join-Path $PSScriptRoot 'scripts/run-a2.py'),'--java-home',$JavaHome,'--gradle-home',$GradleHome)
if($OutputDirectory){$argsList+=@('--output',$OutputDirectory)}
python @argsList
if($LASTEXITCODE -ne 0){throw 'A2 verification failed; pending diagnostics retained.'}
