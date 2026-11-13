@echo off
setlocal
set "RCF_SCRIPT=%~dpn0.py"
set "RCF_PYTHON=%PLAXIS_PYTHON%"
if defined RCF_PYTHON goto run
for /d %%D in ("%ProgramData%\Seequent\PLAXIS Python Distribution*" "%ProgramData%\Bentley\PLAXIS Python Distribution*") do (
  if exist "%%~D\python\python.exe" set "RCF_PYTHON=%%~D\python\python.exe"
  if exist "%%~D\python.exe" set "RCF_PYTHON=%%~D\python.exe"
)
if defined RCF_PYTHON goto run
echo PLAXIS Python was not found in the standard installation folders.
echo Set PLAXIS_PYTHON to the full path of the Python supplied with your PLAXIS version.
echo Trying the default Python; any error will be saved beside the script.
py "%RCF_SCRIPT%"
goto done
:run
if not exist "%RCF_PYTHON%" (
  echo Python does not exist: "%RCF_PYTHON%"
  goto done
)
echo Using Python: "%RCF_PYTHON%"
"%RCF_PYTHON%" "%RCF_SCRIPT%"
:done
pause
endlocal
