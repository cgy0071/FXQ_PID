::@echo off 

@for /d /r %%c in (Listings,Objects) do @if exist %%c ( rd /s /q "%%c" & echo     ɾ��Ŀ¼%%c) 

@for /r  %%c in (*.tmp,*.bak ,*.dep,*.sfr,Backup* ) do del "%%c"
pause