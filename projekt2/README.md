# STERO
## Blok manipulacyjny
### Projekt 2

#### Opis zadania
Napisanie programu sterującego robotem Velma, który spowoduje, że robot chwyci za uchwyt drzwiczek szafki i ją otworzy.

#### Środowisko
Środowisko składa sie ze stolika oraz stojącej na niej szafki z wyłamywalnym uchwytem.

#### Instrukcja uruchomienia
W celu uruchomienia symulacji należy w oddzielnych terminalach wykonać po kolei następujące komendy:

##### Przygotowanie środowiska
* `roscore`  - włączenie roscora 
* `rosrun velma_common reset_shm_comm` - wyczyszczenie kanałów komunikacyjnych
* `roslaunch projekt2 velma_system_stero.launch` - włączenie robota Velma razem z zapisanym swiatem
* `rosrun rviz rviz -d $(rospack find projekt2)/data/velma.rviz` - włączenie programu RVIZ razem z odpowiednia konfiguracja
* `roslaunch rcprg_gazebo_utils gazebo_client.launch` - włączenie GUI Gazebo
* `rosrun velma_task_cs_ros_interface initialize_robot.py` - inicjalizacja robota

##### Włączenie programu do przekładania obiektu
* `roslaunch projekt2 gazebo_tf_publish.launch` - uruchomienie publikacji pozycji szafki, klamki i zawiasu
* `rosrun projekt2 open_cabinet.py` - program do otwierania drzwi

#### Opis działania _gazebo_tf_publish.launch_
Program pobiera pozycje bezpośrednio z Gazebo pozycje szafki, drzwi które robot ma otworzyć oraz zawiasu wokół którego mają się obrócić te drzwi.
Pozycje te są udostępniane do rviza oraz później wykorzystywane w programie _open_cabinet.py_

#### Opis działania _open_cabinet.py_
1. Sprawdzenie pozycji robota. Jeśli robot nie jest w wymaganym stanie początkowym, to wykonywany jest ruch w przestrzeni stawów do pozycji 'zerowej'.
1. Obórcenie korpusu w stronę szafki.
1. Ruch prawej ręki w przestrzeni stawów nad stół.
1. Przeniesienie układu odniesienia narzędzia do dłoni.
1. Ułożenie palców do chwycenia szafki.
1. Ruch w przestrzeni kartezjańskiej w stronę uchwytu.
1. Oparcie ręki o uchwyt.
1. Obliczenie odleglości uchwytu od zawiasu na podstawie pozycji elementów szafki publikowanych przez Gazebo.
1. Interpolacja ruchu końcówki po okręgu, którego promieniem jest odległość z punktu wyżej. Otworzenie szafki o 90 stopni.
1. Wycofanie ręki z 'kolizji z otoczeniem' - odsunięcie jej od uchwytu.
1. Ruch ręki w przestrzeni stawów do pozycji z ręką nad stołem, jednak bezpiecznie odsuniętą od szafki.
1. Otwarcie dłoni.
1. Powrót do pozycji początkowej w przestrzeni stawów.


#### Algorytm otwierania drzwi
Żeby otworzyć drzwi trzeba najpierw przybliżyć dłoń robota do uchwytu a następnie wykonać ruch po okręgu wokół zawiasu drzwi. Do wykonania tego ruchu wyznaczane są równomiernie stany pośrednie (układy TF) do których ma dotrzeć ręka Velmy. Robione jest to w ten sposób że wyznaczana jest pozycja uchwytu względem zawiasu, a nstępnie układ ten jest obracany o określony kąt, aż do osiągnięcia zamierzonej wartości (w naszym przypadku 90 stopni).

```python
def interpolateDoorOpening(velma, totalAngle, degStep):
    #wyliczenie wszystkich stopnie o ktore maja sie obrocic drzwi
    degrees = list(range(degStep, totalAngle, degStep))
    if degrees[-1] != totalAngle:
        degrees.append(totalAngle)
    
    T_B_Zawias = velma.getTf("B", "zawias")

    #obliczenie orignalnej pozycji chwycenia klamki wzgledem zawiasu
    T_zawias_klamka = velma.getTf("zawias", "klamka")
    T_zawias_grasp_org = T_zawias_klamka * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.007, -0.035, +0.07))
    T_zawias_grasp_org = T_zawias_grasp_org * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))

    #Lista gotowych TF-ow
    TFs = []

    for deg in degrees:
        #obrocenie zawiasu o okreslony stopien
        T_B_Z_rot = T_B_Zawias * PyKDL.Frame(PyKDL.Rotation.RotZ(radians(deg)), PyKDL.Vector())
        T_B_grasp = T_B_Z_rot * T_zawias_grasp_org
        TFs.append(T_B_grasp)

    return TFs
```
