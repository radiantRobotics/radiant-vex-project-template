@echo off
echo Updating Radiant VEX Library...
git fetch radiant-vex-lib
git subtree pull --prefix=lib/radiant-vex-lib radiant-vex-lib main --squash
pause
