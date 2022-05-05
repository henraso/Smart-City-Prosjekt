Dette er hovedprogrammene som har noe å gjøre med Zumoen.  

RunFirstOnce setter opp EEPROM riktig og skal lastes opp og kjøres en gang.  
Når programmet er ferdig skal det skrive til serial: "Done! 0".  

Etter du har fått denne responsen kan du kjøre ZumoMain, og da kommer all nødvendig  
data til å være riktig og forbli lagret uansett om Zumoen er av eller på.  

ZumoESPMainFinal er programmet som kjører på ESP32-en, og kommuniserer med Zumo32U4.

ZumoHakkDeteksjon er ikke en komplett kode. Grunnen til vi har valgt å legge den ved her '
er at vi fjernet hakkdetekteringskoden fra hovedprogrammet (ZumoMain). Koden ZumoHakkDeteksjon
er heller ikke godt kommentert, og det er igjen av samme grunn det er ikke en ferdig kode.
