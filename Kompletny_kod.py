import cv2
import numpy as np
from pymycobot import MyCobot280  # Importujemy bibliotekę do obsługi robota MyCobot
import time

# Inicjalizacja połączenia z robotem MyCobot280 na odpowiednim porcie COM
mc = MyCobot280('COM3')  # Jeśli połączenie nie inicjalizuje się poprawnie sprawdź na którym porcie podłączony jest robot w menedżerze urządzeń i zmień numer portu na odpowiedni

def pobranie_czerwonego(): # Funkcja realizująca pobranie oraz odłożenie przedmiotu koloru czerwonego 
    # Ustawienie odniesienia robota do układu współrzędnych 0 - końcówka robota , 1 - narzędzie wykonawcze - efektor
    mc.set_end_type(1)
    # Ustawienie punktu odniesienia robota na efektor końcowy - narzędzie wykonawcze
    mc.set_reference_frame(1)
    # Zmiana koloru głowicy robota na kolor czerwony celem powiadomienia użytkowników o aktualnie wykonywanym ruchu 
    mc.set_color(255,0,0)
    # Ustawienie wartości momentu chwytania chwytaka na wartość maksymalną
    mc.set_HTS_gripper_torque(980)

    # Ustawienie robota do skalibrowanych pozycji zerowych oraz rozwarcie chwytaka
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    time.sleep(1) # Przerwa w programie na podany czas w tym przypadku jedną sekundę
    mc.set_gripper_state(0,70)
    time.sleep(1)

    # Parametry TCP ustalamy na podstawie wymiarów chwytaka (110mm długości w osi)
    # Ustawianie TCP odbywa się poprzez użycie funkcji mc.set_tool_reference w której wprowadzamy tablicę danych przesunięć [x , y, z, Rx, Ry, Rz] - wartość długościu chwytaka - 110mm =>
    # wprowadzamy na miejsce z w tablicy ponieważ tylko to daje oczekiwany efekt ustalenia TCP
    mc.set_tool_reference([0.0 , 0.0, 110.0, 0, 0, 0])

    # Funkcja przenoszenia koloru czerwonego rozbita na poszczególne ruchy (względem ustalonego TCP)
    # Współrzędne docelowe przesunięcia robota wprowadzone jako tablica [x , y, z, Rx, Ry, Rz] , współrzędne oparte o położenie przedmiotów na stanowisku (forma , przedmiot pracy robota)
    wspolrzedne_docelowe = [-65.0, 240.0, 70.0, -90.0, 180.0, 0.0]  
    # Wysyłanie robota do celu z uwzględnieniem ustalonego przesunięcia punktu TCP
    print("Przemieszczanie robota... w pozycję 1") #Wyświetlenie komunikatu o rozpoczęciu ruchu robota 
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe, 10)  # Ruch kątowy z prędkością 10 w wprowadzone wcześniej współrzędne
    print("Przemieszczono robota w pozycję 1")
    time.sleep(1)

    wspolrzedne_docelowe2 = [-65.0, 340.0, 70.0, -90.0, 180.0, 0.0]
    print("Przemieszczanie robota... w pozycję 2")
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe2, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 2")
    time.sleep(1)
    mc.set_gripper_state(1,70)
    time.sleep(1)

    wspolrzedne_docelowe3 = [-65.0, 340.0, 140.0, -90.0, 180.0, 0.0]
    print("Przemieszczanie robota... w pozycję 3")
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe3, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 3")
    time.sleep(1)

    wspolrzedne_docelowe4 = [-65.0, 230.0, 180.0, -90.0, 180.0, 0.0]
    print("Przemieszczanie robota... w pozycję 4")
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe4, 5,1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 4")
    time.sleep(1)

    # Obrót robota w joincie pierwszym powodujący przesunięcie efektora na przeciwną stronę stanowiska
    mc.send_angle(1,55,10)
    time.sleep(4)

    wspolrzedne_docelowe5 = [-50.0, -290.0, 140.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 6")
    mc.sync_send_coords(wspolrzedne_docelowe5, 10)  # Ruch kątowy z prędkością 10 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 6")
    time.sleep(1)

    wspolrzedne_docelowe6 = [-50.0, -340.0, 120.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 6")
    mc.sync_send_coords(wspolrzedne_docelowe6, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 6")
    time.sleep(1)

    wspolrzedne_docelowe7 = [-50.0, -340.0, 70.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 7")
    mc.sync_send_coords(wspolrzedne_docelowe7, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 7")
    time.sleep(2)
    mc.set_gripper_state(0,70)
    time.sleep(2)

    wspolrzedne_docelowe8 = [-50.0, -250.0, 70.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 8")
    mc.sync_send_coords(wspolrzedne_docelowe8, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2) 
    print("Przemieszczono robota w pozycję 8")
    time.sleep(1)

    # Powrót robota do pozycji zerowej oraz otwarcie chwytaka umożliwiając wykonanie następnych ruchów
    mc.send_angles([0, 0, 0, 0, 0, 0], 10)
    mc.set_gripper_state(0,70)
    time.sleep(1)

def pobranie_zielonego(): # Funkcja realizująca pobranie oraz odłożenie przedmiotu koloru zielonego
    # Ustawienie odniesienia robota do układu współrzędnych 0 - końcówka robota , 1 - narzędzie wykonawcze - efektor
    mc.set_end_type(1)
    # Ustawienie punktu odniesienia robota na efektor końcowy - narzędzie wykonawcze
    mc.set_reference_frame(1)
    # Zmiana koloru głowicy robota na kolor zielony celem powiadomienia użytkowników o aktualnie wykonywanym ruchu 
    mc.set_color(0,255,0)
    # Ustawienie wartości momentu chwytania chwytaka na wartość maksymalną
    mc.set_HTS_gripper_torque(980)

    # Ustawienie robota do skalibrowanych pozycji zerowych oraz rozwarcie chwytaka
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    time.sleep(1) # Przerwa w programie na podany czas w tym przypadku jedną sekundę
    mc.set_gripper_state(0,70)
    time.sleep(1)

    # Parametry TCP ustalamy na podstawie wymiarów chwytaka (110mm długości w osi)
    # Ustawianie TCP odbywa się poprzez użycie funkcji mc.set_tool_reference w której wprowadzamy tablicę danych przesunięć [x , y, z, Rx, Ry, Rz] - wartość długościu chwytaka - 110mm =>
    # wprowadzamy na miejsce z w tablicy ponieważ tylko to daje oczekiwany efekt ustalenia TCP
    mc.set_tool_reference([0.0 , 0.0, 110.0, 0, 0, 0])

    # Funkcja przenoszenia koloru czerwonego rozbita na poszczególne ruchy (względem ustalonego TCP)
    # Współrzędne docelowe przesunięcia robota wprowadzone jako tablica [x , y, z, Rx, Ry, Rz] , współrzędne oparte o położenie przedmiotów na stanowisku (forma , przedmiot pracy robota)
    wspolrzedne_docelowe = [0.0, 240.0, 70.0, -90.0, 180.0, 0.0]  
    # Wysyłanie robota do celu z uwzględnieniem ustalonego przesunięcia punktu TCP
    print("Przemieszczanie robota... w pozycję 1") #Wyświetlenie komunikatu o rozpoczęciu ruchu robota 
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe, 10)  # Ruch kątowy z prędkością 10 w wprowadzone wcześniej współrzędne
    print("Przemieszczono robota w pozycję 1")
    time.sleep(1)

    wspolrzedne_docelowe2 = [0.0, 340.0, 70.0, -90.0, 180.0, 0.0]
    print("Przemieszczanie robota... w pozycję 2")
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe2, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 2")
    time.sleep(1)
    mc.set_gripper_state(1,70)
    time.sleep(1)

    wspolrzedne_docelowe3 = [0.0, 340.0, 140.0, -90.0, 180.0, 0.0]
    print("Przemieszczanie robota... w pozycję 3")
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe3, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 3")
    time.sleep(1)

    wspolrzedne_docelowe4 = [0.0, 230.0, 180.0, -90.0, 180.0, 0.0]
    print("Przemieszczanie robota... w pozycję 4")
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe4, 5,1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 4")
    time.sleep(1)

    # Obrót robota w joincie pierwszym powodujący przesunięcie efektora na przeciwną stronę stanowiska
    mc.send_angle(1,55,10)
    time.sleep(4)

    wspolrzedne_docelowe5 = [15.0, -290.0, 140.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 6")
    mc.sync_send_coords(wspolrzedne_docelowe5, 10)  # Ruch kątowy z prędkością 10 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 6")
    time.sleep(1)

    wspolrzedne_docelowe6 = [15.0, -340.0, 120.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 6")
    mc.sync_send_coords(wspolrzedne_docelowe6, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 6")
    time.sleep(1)

    wspolrzedne_docelowe7 = [15.0, -340.0, 70.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 7")
    mc.sync_send_coords(wspolrzedne_docelowe7, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 7")
    time.sleep(2)
    mc.set_gripper_state(0,70)
    time.sleep(2)

    wspolrzedne_docelowe8 = [15.0, -250.0, 70.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 8")
    mc.sync_send_coords(wspolrzedne_docelowe8, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2) 
    print("Przemieszczono robota w pozycję 8")
    time.sleep(1)

    # Powrót robota do pozycji zerowej oraz otwarcie chwytaka umożliwiając wykonanie następnych ruchów
    mc.send_angles([0, 0, 0, 0, 0, 0], 10)
    mc.set_gripper_state(0,70)
    time.sleep(1)
    
def pobranie_niebieskiego(): # Funkcja realizująca pobranie oraz odłożenie przedmiotu koloru niebieskiego
    # Ustawienie odniesienia robota do układu współrzędnych 0 - końcówka robota , 1 - narzędzie wykonawcze - efektor
    mc.set_end_type(1)
    # Ustawienie punktu odniesienia robota na efektor końcowy - narzędzie wykonawcze
    mc.set_reference_frame(1)
    # Zmiana koloru głowicy robota na kolor niebieski celem powiadomienia użytkowników o aktualnie wykonywanym ruchu 
    mc.set_color(0,0,255)
    # Ustawienie wartości momentu chwytania chwytaka na wartość maksymalną
    mc.set_HTS_gripper_torque(980)

    # Ustawienie robota do skalibrowanych pozycji zerowych oraz rozwarcie chwytaka
    mc.send_angles([0, 0, 0, 0, 0, 0], 20)
    time.sleep(1) # Przerwa w programie na podany czas w tym przypadku jedną sekundę
    mc.set_gripper_state(0,70)
    time.sleep(1)

    # Parametry TCP ustalamy na podstawie wymiarów chwytaka (110mm długości w osi)
    # Ustawianie TCP odbywa się poprzez użycie funkcji mc.set_tool_reference w której wprowadzamy tablicę danych przesunięć [x , y, z, Rx, Ry, Rz] - wartość długościu chwytaka - 110mm =>
    # wprowadzamy na miejsce z w tablicy ponieważ tylko to daje oczekiwany efekt ustalenia TCP
    mc.set_tool_reference([0.0 , 0.0, 110.0, 0, 0, 0])

    # Funkcja przenoszenia koloru niebieskiego rozbita na poszczególne ruchy (względem ustalonego TCP)
    # Współrzędne docelowe przesunięcia robota wprowadzone jako tablica [x , y, z, Rx, Ry, Rz] , współrzędne oparte o położenie przedmiotów na stanowisku (forma , przedmiot pracy robota)
    wspolrzedne_docelowe = [65.0, 240.0, 70.0, -90.0, 180.0, 0.0]  
    # Wysyłanie robota do celu z uwzględnieniem ustalonego przesunięcia punktu TCP
    print("Przemieszczanie robota... w pozycję 1") #Wyświetlenie komunikatu o rozpoczęciu ruchu robota 
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe, 10)  # Ruch kątowy z prędkością 10 w wprowadzone wcześniej współrzędne
    print("Przemieszczono robota w pozycję 1")
    time.sleep(1)

    wspolrzedne_docelowe2 = [65.0, 340.0, 70.0, -90.0, 180.0, 0.0]
    print("Przemieszczanie robota... w pozycję 2")
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe2, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 2")
    time.sleep(1)
    mc.set_gripper_state(1,70)
    time.sleep(1)

    wspolrzedne_docelowe3 = [65.0, 340.0, 140.0, -90.0, 180.0, 0.0]
    print("Przemieszczanie robota... w pozycję 3")
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe3, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 3")
    time.sleep(1)

    wspolrzedne_docelowe4 = [65.0, 230.0, 180.0, -90.0, 180.0, 0.0]
    print("Przemieszczanie robota... w pozycję 4")
    time.sleep(1)
    mc.sync_send_coords(wspolrzedne_docelowe4, 5,1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 4")
    time.sleep(1)

    # Obrót robota w joincie pierwszym powodujący przesunięcie efektora na przeciwną stronę stanowiska
    mc.send_angle(1,55,10)
    time.sleep(4)

    wspolrzedne_docelowe5 = [80.0, -290.0, 140.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 6")
    mc.sync_send_coords(wspolrzedne_docelowe5, 10)  # Ruch kątowy z prędkością 10 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 6")
    time.sleep(1)

    wspolrzedne_docelowe6 = [80.0, -340.0, 120.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 6")
    mc.sync_send_coords(wspolrzedne_docelowe6, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 6")
    time.sleep(1)

    wspolrzedne_docelowe7 = [80.0, -340.0, 70.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 7")
    mc.sync_send_coords(wspolrzedne_docelowe7, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2)
    print("Przemieszczono robota w pozycję 7")
    time.sleep(2)
    mc.set_gripper_state(0,70)
    time.sleep(2)

    wspolrzedne_docelowe8 = [80.0, -250.0, 70.0, 90.0, 0.0, 0.0]
    time.sleep(1)
    print("Przemieszczanie robota... w pozycję 8")
    mc.sync_send_coords(wspolrzedne_docelowe8, 5, 1)  # Ruch liniowy z prędkością 5 w wprowadzone wcześniej współrzędne
    time.sleep(2) 
    print("Przemieszczono robota w pozycję 8")
    time.sleep(1)

    # Powrót robota do pozycji zerowej oraz otwarcie chwytaka umożliwiając wykonanie następnych ruchów
    mc.send_angles([0, 0, 0, 0, 0, 0], 10)
    mc.set_gripper_state(0,70)
    time.sleep(1)
    
# Funkcja sprawdzająca czy środek wykrytego okręgu znajduje się w obszarze określonym zakresami wartości położenia x oraz y wyznaczonych na podstawie testów
def w_zasiegu(srodek, zakres_x, zakres_y):
    x, y = srodek
    return zakres_x[0] <= x <= zakres_x[1] and zakres_y[0] <= y <= zakres_y[1]

# Zakresy obszaru, w którym okręgi muszą się znajdować - wartości zostały ustalone w oparciu o testy na stanowisku , poruszenie kamery bądź obrócenie jej spowoduje zmianę otrzymywanych
# współrzędnych , jeśli program wykrywa kolory lecz nie wywołuje ruchu robota najprawdopodobniej trzeba dostosować wartości zakresu wykrytego środka przedmiotu
zakres_czerwony_x = (415, 445)
zakres_czerwony_y = (200, 350)
zakres_zielony_x = (315, 350)
zakres_zielony_y = (200, 350)
zakres_niebieski_x = (200, 245)
zakres_niebieski_y = (200, 350)


# Zmienna stanowiąca o tym czy aktualnie wykonywany jest ruch robota czy robot jest w spoczynku , jesli robot jest w ruchu zmienna wykorzystywana jest do blokady wysyłania kolejnych komend ruchowych do robota
czy_jest_ruch = False 

# Stany bazowe dla pinów GPIO
stany_bazowe_pinu = {19: 0, 22: 0, 23: 0}
# Stan aktualny pinów - stan wykrywania kolorów w zależności od aktualnego stanu na GPIO
stan_aktualny_pinu= {19: 0, 22: 0, 23: 0}

# Funkcja do wykrywania okrągłych obiektów w danym kolorze
def wykryj_kolor_kontury(obraz, dolny_zakres_koloru, gorny_zakres_koloru):
    # Konwersja obrazu z BGR do przestrzeni barw HSV celem przetworzenia obrazu
    hsv = cv2.cvtColor(obraz, cv2.COLOR_BGR2HSV)
    # Tworzenie maski dla danego koloru pozwalającej na izolacje przedmiotu o danym kolorze - piksele mieszczące się w zakresie koloru są zwracane jako białe (1) , reszta jako czarne
    maska = cv2.inRange(hsv, dolny_zakres_koloru, gorny_zakres_koloru)

    # Filtracja szumów przetwarzania obrazu
    # Typowa macierz używana podczas operacji morfoligcznych 
    kernel = np.ones((5, 5), np.uint8)
    # Wypełnia małe dziury w białych obszarach maski 
    maska = cv2.morphologyEx(maska, cv2.MORPH_CLOSE, kernel)
    # Usuwa małe białe szumy poza obszarami docelowymi
    maska = cv2.morphologyEx(maska, cv2.MORPH_OPEN, kernel)

    # Znajdowanie konturów w obrazie z nałożoną maską po operacjach morfologicznych jako wynik zwraca listę konturów , czyli ich punkty tworzące dany kształt (okrąg)
    kontury, _ = cv2.findContours(maska, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    wykryte_okregi = []

    # Przeszukiwanie listy konturów tworzących obraz w obrazie binarnym
    for kontur in kontury:
        # Filtracja konturów na podstawie wielkości konturu - oblicza powierzchnię konturu również filtruje małe obiekty (szumy) pozostawiając tylko większe obszary
        if cv2.contourArea(kontur) > 500:  # Próg minimalnej wielkości konturu
            (x, y), promien = cv2.minEnclosingCircle(kontur) # Znajduje najmniejszy okrąg otaczający kontur o minimalnej wielkości
            srodek = (int(x), int(y)) # Współrzędne środka okręgu
            promien = int(promien) # Promień okręgu

            # Sprawdzenie, czy kontur przypomina okrąg (na podstawie proporcji)
            pole_konturu = cv2.contourArea(kontur) # Powierzchnia obliczona na podstawie konturu
            pole_okregu = np.pi * (promien ** 2) # Powierzchnia idealnego okręgu o znalezionym promieniu
            okraglosc = pole_konturu / pole_okregu # Stosunek pola konturu do pola okręgu
            if 0.7 < okraglosc < 1.2:  # Tolerancja dla okrągłości , jeśli okrąg mieści się w tolerancji kontur uznawany jest za okrąg
                wykryte_okregi.append((srodek, promien)) # Wykryty okrąg dodawany jest do listy wynikowej z której później pobierane są wartości srodka oraz promienia

    return wykryte_okregi #Funkcja zwraca wykryte okręgi wraz z współrzędnymi środka okręgu oraz jego promieniem , srodek oraz promien zostają wykorzystane później do rysowania okręgów na obrazie

# Zainicjalizowanie kamery w programie po indeksie oznaczonym w systemie
kamera = cv2.VideoCapture(0)  # Indeks 0 oznacza pierwszą kamerę w systemie

if not kamera.isOpened():
    print("Nie udało się otworzyć kamery!")
    exit()

# Pętla while realizacji funkcji odczytu obrazu z kamery oraz rysowania okręgów
while True:
    # Odczyt obrazu z kamery klatka po klatce
    ret, obraz = kamera.read() # ret oznacza czy odczyt był udany , obraz to pojedyncza klatka odczytana z kamery
    if not ret: # jeśli odczyt klatki się nie powiedzie , program przerwa pętlę
        print("Nie udało się odczytać obrazu z kamery!")
        break

    # Zakresy kolorów w przestrzeni HSV definiuje dolne i górne granice kolorów które mają być wykrywane
    # Zakresy koloru czerwonego , rozdzielone na dwa zakresy ze względu na naturę czerwonego w przestrzeni HSV
    dolny_czerwony1 = np.array([0, 120, 70])
    gorny_czerwony1 = np.array([10, 255, 255])
    dolny_czerwony2 = np.array([170, 120, 70])
    gorny_czerwony2 = np.array([180, 255, 255])
    # Zakresy koloru zielonego
    dolny_zielony = np.array([25, 40, 20])
    gorny_zielony = np.array([105, 255, 255])
    # Zakresy koloru niebieskiego
    dolny_niebieski = np.array([90, 60, 50])
    gorny_niebieski = np.array([130, 255, 255])

    # Odczyt stanów GPIO z robota w pętli for
    for pin in [19, 22, 23]: # Odczyt stanów pinu 19 , 22 , 23 w pętli for
        stan_pinu = mc.get_basic_input(pin) # Funkcja ta odczytuje bieżący stan wejścia sprawdzanego pinu np 19 , 1 = stan wysoki , 0 = stan niski
        if stan_pinu == 1 and stany_bazowe_pinu[pin] == 0:  # Wykrycie stanu wysokiego na wejściu portu GPIO
            stan_aktualny_pinu[pin] = 1 - stan_aktualny_pinu[pin]  # Jeśli wykryto przejście stanu pinu z niskiego na wysoki funkcja zasetuje nam pin , utrzymując jego stan wysoki

        stan_aktualny_pinu[pin] = stan_pinu

    # Wykrywanie obiektów na podstawie stanów pinów - portów GPIO
    #Jeśli aktywowany jest port GPIO 19 załączy się funkcja wykryj_kolor dla zakresów koloru czerwonego
    if not czy_jest_ruch:
        if stan_aktualny_pinu[19]:
             # Użycie funkcji wykryj_kolor i przypisanie do nich wartości zakresów HSV
            czerwone_okregi1 = wykryj_kolor_kontury(obraz, dolny_czerwony1, gorny_czerwony1)
            czerwone_okregi2 = wykryj_kolor_kontury(obraz, dolny_czerwony2, gorny_czerwony2)
            czerwone_okregi = czerwone_okregi1 + czerwone_okregi2 # Wynik działania funkcji jest sumowany ponieważ mamy dwa zakresy koloru czerwonego

            for srodek, promien in czerwone_okregi:
                cv2.circle(obraz, srodek, promien, (0, 0, 255), 2) # Funkcja rysuje okręgi wokół wykrytych srodkow , promieniem wykrytym na podstawie nałożonej maski - rysuje okrąg na obwodzie
                cv2.circle(obraz, srodek, 5, (0, 0, 255), -1) # Funkcja rysuje mały okrąg na środku obiektu poprzez jego wyznaczony srodek , promieniem 5 
                print('czerwony :', srodek)
                if w_zasiegu(srodek, zakres_czerwony_x, zakres_czerwony_y):
                    pobranie_czerwonego()
                    ruch_aktywowany = True
                    break

    #Jeśli aktywowany jest port GPIO 22 załączy się funkcja wykryj_kolor dla zakresów koloru zielonego
        if stan_aktualny_pinu[22]:
            # Użycie funkcji wykryj_kolor i przypisanie do nich wartości zakresów HSV
            zielone_okregi = wykryj_kolor_kontury(obraz, dolny_zielony, gorny_zielony)
            for srodek, promien in zielone_okregi:
                cv2.circle(obraz, srodek, promien, (0, 255, 0), 2) # Funkcja rysuje okręgi wokół wykrytych srodkow , promieniem wykrytym na podstawie nałożonej maski - rysuje okrąg na obwodzie
                cv2.circle(obraz, srodek, 5, (0, 255, 0), -1) # Funkcja rysuje mały okrąg na środku obiektu poprzez jego wyznaczony srodek , promieniem 5 
                print('zielony :', srodek)
                if w_zasiegu(srodek, zakres_zielony_x, zakres_zielony_y):
                    pobranie_zielonego()
                    ruch_aktywowany = True
                    break

    #Jeśli aktywowany jest port GPIO 23 załączy się funkcja wykryj_kolor dla zakresów koloru niebieskiego
        if stan_aktualny_pinu[23]:
            # Użycie funkcji wykryj_kolor i przypisanie do nich wartości zakresów HSV
            niebieskie_okregi = wykryj_kolor_kontury(obraz, dolny_niebieski, gorny_niebieski)
            for srodek, promien in niebieskie_okregi:
                cv2.circle(obraz, srodek, promien, (255, 0, 0), 2) # Funkcja rysuje okręgi wokół wykrytych srodkow , promieniem wykrytym na podstawie nałożonej maski - rysuje okrąg na obwodzie
                cv2.circle(obraz, srodek, 5, (255, 0, 0), -1) # Funkcja rysuje mały okrąg na środku obiektu poprzez jego wyznaczony srodek , promieniem 5 
                print('niebieski :', srodek)
                if w_zasiegu(srodek, zakres_niebieski_x, zakres_niebieski_y):
                    pobranie_niebieskiego()
                    ruch_aktywowany = True
                    break

    # Wyświetlanie obrazu z podłączonej kamery w czasie rzeczwysitym pod numerem 0 w systemie
    cv2.imshow("Analiza kolorow R, G ,B", obraz)

    # Warunek którego aktywacja zakończy działanie przetwarzania obrazu - wywołanie poprzez wciśnięcie klawisza q
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Zwolnienie kamery i zamknięcie okienek
kamera.release()
cv2.destroyAllWindows()
