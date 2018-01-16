## Система управления для робота-машинки
Проект включает в себя четыре части:
* Система управления роботом-машинкой
* Система технического зрения для проверки точности выполняемых маневров
* Модель в [Scilab](https://github.com/opencollab/scilab)
* Подробный [отчет](https://github.com/kirillin/parking-lego-car/blob/master/report/build/main.pdf) с результатами моделирования и испытаний 

### Установка

1. Собирите робота-машинку ([инструкция]()) из [LEGO mindstorms EV3](https://www.lego.com/ru-ru/mindstorms); 
2. Загрузите образ с ОС Linux в соответствии с [инструкцией](http://www.ev3dev.org/docs/getting-started/) на сайте сообщества [ev3dev](http://www.ev3dev.com);
3. Подключите к контроллеру EV3 WiFi модуль и подключитесь к ваше сети;
4. Подключитесь по `ssh` к роботу (пользователь: robot, пароль: maker):
    ```bash
    ssh robot@{ip_адрес_робота}
    ```
5. Склонируйте на робота репозиторий:
    ```bash 
    cd ~
    git clone https://github.com/kirillin/parking-lego-car.git     
    ```
6. Дайте права на запуск скриптам:
    ```bash
    cd ~/parking-lego-car/sources/carsystems/
    sudo chmod +x *.py
    ```

## Запуск

В качестве примера запустим один из тестовых скриптов. Доступны два способа запуска:
1. По `shh` непосредственно с самого робота (необходимо WiFi подключение):
    ```bash
    cd ~/parking-lego-car/sources/carsystems/
    python3 test_trajectory_controller.py
    ```
2. Из меню самого контроллера EV3 (в меню фалового менеджера найдите и запустите скрипт `test_trajectory_controller.py`).

В результате запуска, робот должен поехать по траектории, как на рисунке:
![Иллюстрация к проекту](https://github.com/kirillin/parking-lego-car/raw/master/report/images/cv_6.png)

## Authors

* **Evgeniy Antonov** - [mrclient](https://github.com/mrclient)
* **Kirill Artemov** - [kirillin](https://github.com/kirillin)

## License

