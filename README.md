# nto-2023 (Команда FWT)
## 1 день
Получили десятый коптер. Настройка, установка образа. Замена неисправной Raspberry Pi, был скол возле порта type-c. Николай в первый день работы напечатал 2 капсулы класса «А» (характеристика одной капсулы для пожаротушения класса “А”: высота 3,5 см, диаметр 1,5 см, форма цилиндрическая. Всего: 4 шт) и 4 капсулы класса «В» (характеристика одной капсулы для пожаротушения класса “B”: кубическая форма со стороной 2,5 см. Всего: 4 шт). Также разработал полезную нагрузку (далее ПН), удовлетворяющую следующим требованиям:
*	модульность;
*	возможность изготовления посредством 3D печати (FDM принтер);
*	возможность осуществления сборки устройства посредством винтового/шпоночного соединения;
*	возможность быстрого монтажа на квадрокоптер и демонтажа соответственно;
*	максимальный вес устройства 200 г. (без учета веса капсул);
*	возможность реализации автономного сброса капсул для тушения возгораний определенного типа;
*	совместимость с программируемым квадрокоптером «COEX Клевер 4 Code»;
*	возможность надежного монтажа на квадрокоптер.

##  2 день
Настройка, перепрошивка и калибровка полётного контроллера. Написали [walls.py](https://github.com/Arseniyyy/nto-2023/blob/main/walls.py). Начали писать `flight.py` и `walls.py`. `flight.py` выполняет тестовые взлёты квадрокоптера, чтобы убедиться в исправности всех деталей. `walls.py` - это основной код, выполняющий определение длин стен. Во второй день Николай приступил к разработке рабочей 3D-модели ПН. В ходе выполненной работы разработал следующие детали:
<p align="center">Основной крепежный элемент:</p>
<img title="a title" alt="Alt text" src="https://raw.githubusercontent.com/Arseniyyy/nto-2023/main/images/1.png">

<p align="center">Ящик для хранения капсул класса «А» или «В»:</p>
<img title="a title" alt="Alt text" src="https://raw.githubusercontent.com/Arseniyyy/nto-2023/main/images/2.jpg">

<p align="center">Шестереночную реечную передачу</p>
<img title="a title" alt="Alt text" src="https://raw.githubusercontent.com/Arseniyyy/nto-2023/main/images/3.jpg">
<img title="a title" alt="Alt text" src="https://raw.githubusercontent.com/Arseniyyy/nto-2023/main/images/4.jpg">

<p align="center">Пластину для крепления частей рейки</p>
<img title="a title" alt="Alt text" src="https://raw.githubusercontent.com/Arseniyyy/nto-2023/main/images/5.jpg">

<p align="center">Общую сборку ПН</p>
<img title="a title" alt="Alt text" src="https://raw.githubusercontent.com/Arseniyyy/nto-2023/main/images/6.jpg">
<img title="a title" alt="Alt text" src="https://raw.githubusercontent.com/Arseniyyy/nto-2023/main/images/7.jpg">

Все 3D-модели(.stp; .stl; .ipt) находятся в Google Диске по ссылке: https://drive.google.com/drive/folders/1AfTJMEPyOBrMpUYqz5cL1K3E3qNKNLmr. Также Николай успел поставить на 3D-печать модель “Box_AB.stl” в количестве 2-ух штук.


## 3 день
Замена Raspberry Pi во второй раз, не раздавал wifi. Дальномер начал работать. Получился первый полёт. Написали [flight.py](https://github.com/Arseniyyy/nto-2023/blob/main/flight.py), в котором реализовали пролёт до правого верхнего угла и обратно в ту точку, где был произведён взлёт. В течении этого дня Николай разбирался с 3D-печатью остальных деталей, но ничего не успел напечатать. В перерыве между печатью деталей он модифицировал Microservo MG90S(далее MS): изменил амплитуду поворота качалки с 180 градусов до 360 градусов, тем самым сделав возможным реализацию своей ПН. Позже он протестировал модернизированный MS при помощи программируемого микроконтроллера Arduino Nano следующим программным кодом: 

```c++
#include "Servo.h"
Servo myservo;

void setup() {
  myservo.attach(4);// контакт 4 для управления сервомотором
}

void loop() {
  myservo.write(0); delay(5000);// переключение сервомотора в направление против часовой стрелки в течении 5 секунд
  myservo.write(90); delay(5000);// переключение сервомотора в направление по часовой стрелки в течении 5 секунд
  myservo.write(180); delay(5000);// переключение сервомотора в направление против часовой стрелки в течении 5 секунд
  myservo.write(90); delay(5000);// переключение сервомотора в направление по часовой стрелки в течении 5 секунд
}
```
## 4 день
Коптер смог пролететь до левого верхнего угла. Николай смог допечатать ПН с раза 20, но собрать и закрепить на квадрокоптере в положенный срок не успел, оставив его до финальных испытаний.
