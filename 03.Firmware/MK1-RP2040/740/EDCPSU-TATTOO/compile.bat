
@echo off
:: Board a usar en la compilacion
set board=rp2040:rp2040:rpipico
set libraries="C:/Users/javie/Team Dropbox/JRODRIGUEZ/Repositorios/07.MK2/03.Firmware/Libraries"
:: Comando para compilar y exportar los archivos binarios en la carpeta residente de este .bat
arduino-cli compile -e -t --fqbn %board% -v --libraries %libraries%

