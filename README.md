# Parking_senzori

Uputstvo za upotrebu nalazi se u PDF folderu "Parking_senzori_Uputstvo".


Da bi se u cmd promptu ispisivalo trenutno stanje lijevog senzora svakih 200ms, potrebno je u Lijevi_senzor_task otkomentarisati: 
//xSemaphoreTake(lijevi, portMAX_DELAY);// ceka na serijski prijemni interapt svakih 200ms,

a zakomentarisati:
xSemaphoreTake(RXC_BinarySemaphore_0, portMAX_DELAY);// ceka na serijski prijemni interapt 


Isto tako i za desni senzor potrebno je u Desni_senzor_task otkomentarisati:
//xSemaphoreTake(desni, portMAX_DELAY);// ceka na serijski prijemni interapt svakih 200ms

a zakomentarisati:
xSemaphoreTake(RXC_BinarySemaphore_1, portMAX_DELAY);// ceka na serijski prijemni interapt



Opis thread-ova koji su korišćeni u projektu su:

1.	SerialReceive_Task
Ovaj task učitava primljene karaktere, tj. komandne riječi START/STOP koje se šalju preko kanala 2 (AdvUniCom 2) i šalje tu komandnu riječ tasku za obradu podataka (Obrada_task).

2.	LED_bar_task
Ovaj task služi za ručno uključivanje i isključivanje prekidača i šalje stanje tasku za obradu podataka.

3.	Lijevi_senzor_task
   Ovaj task učitava primljeni karakter koji se šalje preko kanala 0 i kao takav šalje:
    -	tasku za obradu podataka (Obrada_task),
    -	tasku koji predstavlja displej (Seg7_task),
    -	cmd promptu na kom se ispisuje trenutna udaljenost u cm.

4.	Desni_senzor_task
   Ovaj task učitava primljeni karakter koji se šalje preko kanala 1 i kao takav šalje:
    -	tasku za obradu podataka (Obrada_task),
    -	tasku koji predstavlja displej (Seg7_task),
    -	cmd promptu na kom se ispisuje trenutna udaljenost u cm.

5.	Obrada_task
  Ovaj task prima podatke sa:
    -	kanala 0 (AdvUniCom 0), tj. podatke sa lijevog senzora (Lijevi_senzor_task),
    -	kanala 1 (AdvUniCom 1), tj. podatke sa lijevog senzora (Desni_senzor_task),
    -	kanala 2 (AdvUniCom 2), tj. komandnu riječ START/STOP,
    -	stanje prekidača od taska Led_bar_task.

  Podaci se obrađuju na osnovu primljenih podataka sa kanala 2 i stanja prekidača. 
    -	Ako je primljena komandna riječ START ili uključen prekidač podaci sa kanala 0 i kanala 1, tj. lijevog i desnog senzora se obrađuju u skladu sa očitanom udaljenošću i šalju       kanalu 2 da je sistem uključen i udaljenost u postotcima za oba senzora posebno,
    -	Ako je primljena komandna riječ STOP ili prekidač isključen, podaci sa senzora se ne obraćuju i šalje se kanalu 2 da je sistem isključen.

  Ovaj task takođe šalje podatak o stanju sistema, da li je uključen ili isključen, tasku Seg7_task.

6.	SerialSend_Task
  Ovaj task prima obrađene podatke iz taska za obradu podataka (Obrada_task) i ispisuje trenutno stanje sistema i očitane udaljenosti u postotcima sa lijevog i desnog senzora na     kanal 2 (AdvUniCom 2).

7.	Seg7_task
  Ovaj task prima podatke od:
    -	taska za obradu podataka (Obrada_task) o stanju sistema (uključen/isključen),
    -	sa kanala 0, tj. podatke od lijevog senzora,
    -	sa kanala 1, tj. podatke od desnog senzora,
  te na osnovu tih podataka uključuje/isključuje displej i na njemu ispisuje vrijednosti lijevog i desnog senzora u cm, te ih šalje taskovima Blink_1 i Blink_2.

8.	Blink_1
  Ovaj task prima podatke od Seg7_task o postotcima udaljenosti koje očitava lijevi i desni senzor (0%-50% i 50%-100%), te na osnovu toga pali i gasi diode koje predstavljaju tu     udaljenost.

9.	Blink_2
  Ovaj task prima podatke od Seg7_task o postotcima udaljenosti koje očitava lijevi i desni senzor (0%), te na osnovu toga pali i gasi diode koje predstavljaju tu udaljenost.
  Blink_1 i Blink_2 su posebno radjeni jer treptanje dioda koje predstavljaju postotak udaljenosti 0% treba brže da trepere u odnosu na diode koje predstavljaju udaljenost 0%-50%   i 50%-100%.
