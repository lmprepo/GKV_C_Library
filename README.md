# GKV_CLibrary
Данный репозиторий включает библиотеку для приема данных и настройки параметров инерциальных модулей серии ГКВ, а также три консольных примера для Windows и Linux: 
- прием любых данных с инерциального модуля и вывод значений в консоль
- прием данных с инерциального модуля в формате кодов АЦП и вывод значений параметров в консоль.
- выбор алгоритма работы инерциального модуля, прием данных и вывод значений параметров в консоль.
Проект реализован с использованием системы сборки CMake.

Библиотека включает в себя:
- структуры всех используемых инерциальным модулем пакетов (включая пустые структуры).
- парсер принимаемых пакетов.
- основные функции для конфигурирования передаваемых пакетов.
- функции настройки основных параметров инерциального модуля.

Библиотека НЕ работает напрямую с serial-портом. В примерах для работы с serial-портом использованы стандартная библиотека windows и библиотека termios для Linux.
Аналогично может быть использована любая удобная пользователю библиотека serial-порта для используемой платформы.
Обработка принятых пакетов данных и отправка запросов осуществляется с помощью пользовательских callback-функций (см. примеры, функции WriteCOM и RecognisePacket).
