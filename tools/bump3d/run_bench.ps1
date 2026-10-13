# Corre el banco de escenas del bump en Gazebo (motor de MARS).
#
#   .\run_bench.ps1                         genera el mundo, abre la ventana 3D y mide
#   .\run_bench.ps1 -Headless -Seconds 12   sin ventana: solo la tabla de resultados
#   .\run_bench.ps1 -Speeds 1,2 -LiftsCm 0,4
#
# No toca el codigo del robot ni la instalacion de MARS: solo usa sus ejecutables
# (mars-sim-server, mars-sim-gui), su entorno conda y el modelo offseason2026.
param(
  [switch]$Headless,
  [double]$Seconds = 12,
  [string]$Speeds = "0.5,1,2,3",
  [string]$Directions = "intake_adelante,intake_atras,de_lado,diamante_intake,diamante_atras",
  [string]$LiftsCm = "0",
  [string]$MarsSim = "$env:LOCALAPPDATA\Programs\MARS\sim"
)
$ErrorActionPreference = "Stop"
$here = $PSScriptRoot
$build = Join-Path $here "build"

$condaEnv = @($env:MARS_CONDA_ENV, "$env:USERPROFILE\miniforge3\envs\mars-sim", "$env:LOCALAPPDATA\miniforge3\envs\mars-sim",
              "$env:USERPROFILE\miniconda3\envs\mars-sim") |
  Where-Object { $_ -and (Test-Path (Join-Path $_ "python.exe")) } | Select-Object -First 1
if (-not $condaEnv) { throw "No encontre el entorno conda 'mars-sim' (el de MARS Simulation Studio)." }
foreach ($f in @("build\mars-sim-server.exe", "build\mars-sim-gui.exe", "models\offseason2026\mecanismo.json")) {
  if (-not (Test-Path (Join-Path $MarsSim $f))) { throw "Falta $f en $MarsSim" }
}

# Mismo entorno que arma MARS: sin MSYS2 en el PATH, plugins y modelos de MARS.
$env:PATH = "$condaEnv\Library\bin;" + (($env:PATH -split ';' | Where-Object { $_ -and $_ -notmatch 'msys64' }) -join ';')
$env:GZ_SIM_RESOURCE_PATH = "$MarsSim\models"
$env:GZ_SIM_SYSTEM_PLUGIN_PATH = "$MarsSim\build"
# Lo que pone `conda activate` (etc\conda\activate.d): sin OGRE2_RESOURCE_PATH la ventana
# no encuentra el render de OpenGL 3 y se cierra.
$env:CONDA_PREFIX = $condaEnv
$env:OGRE_RESOURCE_PATH = "$condaEnv\Library\bin"
$env:OGRE2_RESOURCE_PATH = "$condaEnv\Library\bin\OGRE-Next"
$env:SDF_PATH = "$condaEnv\Library\share\sdformat13\1.10"
# Particion propia: dos corridas a la vez (o MARS Studio abierto, que tambien usa el mundo
# "mars") no se mezclan los topicos.
$env:GZ_PARTITION = "bump3d_$PID"
$py = "$condaEnv\python.exe"

& $py "$here\gen_bump_bench.py" --mars-sim $MarsSim --speeds $Speeds --directions $Directions --lifts-cm $LiftsCm 2>$null
if ($LASTEXITCODE -ne 0) { throw "fallo gen_bump_bench.py" }
$world = Join-Path $build "bump_bench.sdf"
$meta = Join-Path $build "bump_bench.json"
$results = Join-Path $build "resultados.json"

$procs = @()
try {
  # El medidor se suscribe ANTES de que arranque la fisica para no perder el inicio.
  $medArgs = @("`"$here\measure_bench.py`"", "`"$meta`"", "--seconds", $Seconds,
               "--timeout", ([int]($Seconds * 20 + 60)), "--out", "`"$results`"")
  if (-not $Headless) { $medArgs += @("--unpause-after", "12", "--loop") }
  $med = Start-Process -FilePath $py -ArgumentList $medArgs -NoNewWindow -PassThru `
    -RedirectStandardOutput "$build\medicion.txt" -RedirectStandardError "$build\medicion.err"
  $null = $med.Handle; $procs += $med
  Start-Sleep -Seconds 2

  if ($Headless) {
    $iter = [int][math]::Ceiling(($Seconds + 0.5) / 0.004)
    $srv = Start-Process -FilePath "$MarsSim\build\mars-sim-server.exe" -ArgumentList "`"$world`"", $iter `
      -WorkingDirectory $MarsSim -NoNewWindow -PassThru -RedirectStandardOutput "$build\servidor.txt" -RedirectStandardError "$build\servidor.err"
    $null = $srv.Handle; $procs += $srv
  } else {
    # Con ventana: el servidor arranca PAUSADO y el medidor le da Play 12 s despues, cuando la
    # ventana ya cargo las mallas (asi se ve el cruce desde el principio).
    $srv = Start-Process -FilePath "$MarsSim\build\mars-sim-server.exe" -ArgumentList "`"$world`"" `
      -WorkingDirectory $MarsSim -NoNewWindow -PassThru -RedirectStandardOutput "$build\servidor.txt" -RedirectStandardError "$build\servidor.err"
    $null = $srv.Handle; $procs += $srv
    $gui = Start-Process -FilePath "$MarsSim\build\mars-sim-gui.exe" -ArgumentList "`"$build\gui.config`"" `
      -WorkingDirectory $MarsSim -PassThru -RedirectStandardOutput "$build\gui.txt" -RedirectStandardError "$build\gui.err"
    $null = $gui.Handle; $procs += $gui
    Write-Host "Ventana abierta; arranca sola en ~12 s. La tabla sale cuando la simulacion llega a $Seconds s." -ForegroundColor Cyan
  }
  if ($Headless) {
    $med.WaitForExit()
  } else {
    # En bucle: el medidor reinicia el mundo cada vez que todas las escenas terminan.
    Write-Host "Se repite en bucle. Cierra la ventana de Gazebo para terminar." -ForegroundColor Cyan
    $gui.WaitForExit()
  }
  Get-Content "$build\medicion.txt" | Where-Object { $_ -notmatch '^\[medir\] vuelta' }
  Write-Host "resultados (primera vuelta): $results"
} finally {
  foreach ($p in $procs) { if ($p -and -not $p.HasExited) { Stop-Process -Id $p.Id -Force -ErrorAction SilentlyContinue } }
}
