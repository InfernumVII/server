# server

## Инструкция по запуску:


1. Открыть папку с проектом
2. Запускаете файл start.bat(если потребуется нажмите на необходимые клавиши для авторизации) - он прокидывает файлы на дроны в домашнюю директорию
3. Запускаете run.py
4. Открыть сервер по адресу http://localhost:8000/
5. На сервере выбирете галочками всех дронов и нажимаете START MISSION



## Инструкция для ручного прокидывания и запуска:

1. запустить serverFINAL.py на windows
1.2. ввести ipconfig
1.3 взять IPv4-адрес из Адаптера
1.4 отредактировать ip адрес в asyncdrone.py в строке
uri = "ws://localhost:8000/ws"
2. Прокинуть asyncdrone.py,recognition.py на дрона 1 и на дрона 2
3. запустить asyncdrone.py на дроне 1,  запустить asyncdrone.py на дроне 2
Ссылка на документацию: 
https://docs.google.com/document/d/1bX64CUT7YZSQ90xr5pkMVIGZ3K3FhxtVXV0nX2Am7IU/edit?usp=sharing
