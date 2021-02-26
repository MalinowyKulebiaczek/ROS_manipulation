# STERO
## Blok manipulacyjny
### Projekt 1

#### Opis zadania
Napisanie programu sterującego robotem Velma, który spowoduje, że robot chwyci obiekt znajdujący się na jednym stole i przeniesie go na drugi.

#### Środowisko
Środowisko składa sie z dwóch stolików typu _cafe_table_ oraz obiektu typu _jar_, który stoi na jednym z nich.

#### Instrukcja uruchomienia
W celu uruchomienia symulacji należy w oddzielnych terminalach wykonać po kolei następujące komendy:

##### Przygotowanie środowiska
* `roscore`  - włączenie roscora 
* `rosrun velma_common reset_shm_comm` - wyczyszczenie kanałów komunikacyjnych
* `roslaunch projekt1 velma_system_stero.launch` - włączenie robota Velma razem z zapisanym swiatem
* `rosrun rviz rviz -d $(rospack find projekt1)/data/velma.rviz` - włączenie programu RVIZ razem z odpowiednia konfiguracja
* `rosrun velma_task_cs_ros_interface initialize_robot.py` - inicjalizacja robota
* `roslaunch velma_ros_plugin velma_planner.launch` - uruchomienie planera
* `roslaunch velma_common octomap_offline_server.launch octomap_file:="$(rospack find projekt1)/data/octomap_proj1.ot"` - włączenie zapisanej oktomapy w trybie offline
* `roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:=jar::link frame_id:="sloik"` - publikowanie pozycji słoika - objektu do chwycenia
* `roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:=cafe_table_0::link frame_id:="stol1"` - publikowanie pozycji stołu nr 1
* `roslaunch rcprg_gazebo_utils gazebo_publish_ros_tf_object.launch link_name:=cafe_table::link frame_id:="stol2"` - publikowanie pozycji stołu nr 2

###### Opcjonalnie: 
* `roslaunch rcprg_gazebo_utils gazebo_client.launch` - włączenie GUI Gazebo
* `roslaunch velma_common octomap_server.launch`	- wlaczenie oktomapy online (w przypadku, gdy chcemy sprawdzić bieżący stan otoczenia)
* `rosrun projekt1 explore_environment.py`	- uruchomienie skryptu eksplorującego środowisko

##### Włączenie programu do przekładania obiektu
* `rosrun projekt1 publish_positions.py` - uruchomienie programu do opublikowania pozycji docelowych końcówek robota
* `rosrun projekt1 move_jar.py` - uruchomienie programu do przekładania obiektu

#### Opis działania programu
Nasze rozwiązanie składa się z dwóch programów:
* **publish_positions.py** - program publikujący docelowe pozycje _TF_ końcówek robota, tj.:
  * _pregrasp_1_ - pozycja końcówki tuż przy obiekcie, jest to pozycja, w której robot jest gotowy do zaciśnięcia palców, czyli chwycenia obiektu.
  * _pregrasp_2_ - pozycja końcówki umożliwiająca płynne przejście do pozycji _pregrasp_1_. Jest to również pozycja, do której dąży robot po chwyceniu obiektu, aby umożliwić użycie planera.
  * _predrop_1_ - pozycja końcówki tuż przed odłożeniem obiektu.
  * _afterdrop_1_ - pozycja końcówki do której dąży robot, po odłożeniu obiektu.
* **move_jar.py** - główny program sterujący ruchami robota podczas symulacji - program wykonuje przekładanie słoika

#### Wizualizacja pozycji _TF_ i oktomapy
![Oktomapa i TFy](/docs/img/oktomapa.png)

#### Opis działania _move_jar.py_
1. Sprawdzenie pozycji robota. Jeśli robot nie jest w wymaganym stanie początkowym, to przy użyciu planera wykonywany jest ruch, który przywraca go do wymaganej konfiguracji początkowej stanów.
1. Schowanie palców w celu ułatwienia zadania planerowi.
1. Wykonanie ruchu obu rąk ponad stoliki z użyciem planera w przestrzeni stawów. Pozycja została dobrana eksperymentalnie.
1. Wybór ręki, która będzie przenosić słoik na podstawie jego pozycji.
1. Ruch ręki w kierunku słoika w przestrzeni kartezjańskiej. Najpierw do pozycji _pregrasp_2_, a następnie do _pregrasp_1_
1. Zaciśnięcie palców - chwycenie słoika.
1. Ruch ręki trzymającej słoik do pozycji _pregrasp_2_.
1. Ruch robota w kierunku punktu docelowego przy pomocy planera używając trybu ruchu w przestrzeni stawów.
1. Ruch ręki w kierunku punktu docelowego w przestrzeni kartezjańskiej (do pozycji _predrop_1_) oraz odstawienie obiektu (otworzenie dłoni)
1. Ruch ręki do pozycji _afterdrop_1 w przestrzeni kartezjańskiej.
1. Zamknięcie dłoni.
1. Ruch robota do konfiguracji początkowej z użyciem planera.
1. Otwarcie dłoni.

#### Algorytm chwytania słoika
Gdy ręka robota jest w pozycji gotowej do chwycenia obiektu wywoływana jest funkcja, której fragment przedstawiono poniżej. Na jej początku ręka przechodzi do pozycji, w której palce są zaciśnięte o ilość stopni wyrażoną w argumencie _init_deg_ (wartość dobrana eksperymentalnie, palce nie dotykają jeszcze w tej konfiguracji obiektu). Następnie ręka zaciska się stopniowo co 2 stopnie do momentu, gdy pozycja palców nie będzie się zgadzać z konfiguracją oczekiwaną, co ilustruje poniższy fragment kodu. Oznacza to, że chwycilismy słoik i robot jest gotowy do jego podniesienia.

```python
def graspJar(velma, init_deg, hand):
(...)
  while(isHandConfigurationClose(currentHandConfiguration, dest_q, tolerance=0.1)):
        i += 2
        dest_q = [deg2rad(init_deg+i), deg2rad(init_deg+i), deg2rad(init_deg+i), deg2rad(0)]

        if hand == 'left':
            velma.moveHandLeft(dest_q, [1, 1, 1, 1], [4000, 4000, 4000, 4000], 1000, hold=False)
            if velma.waitForHandLeft() != 0:
                exitError(2)
            currentHandConfiguration = velma.getHandLeftCurrentConfiguration()
        elif hand == 'right':
            velma.moveHandRight(dest_q, [1, 1, 1, 1], [4000, 4000, 4000, 4000], 1000, hold=False)
            if velma.waitForHandRight() != 0:
                exitError(2)
            currentHandConfiguration = velma.getHandRightCurrentConfiguration()
(...)
```

#### Dobór pozycji docelowej słoika na drugim stoliku
Pozycja, w której odkładany jest słoik, jest wyznaczana na podstawie pozycji stolika względem robota. Pozycja ta jest wyznaczana na podstawie pozycji środka stolika i jest przesunięta nieco w kierunku robota, aby nie musiał sięgać zbyt daleko.
