# Generator
Tar lyd som input og sender ut 5 signaler basert på FFT.

## Errata

### Versjon 1.0
- Mangler merking av +/- på power inputs



## Output fra shield med Magnus sin firmware
På oppstart - S1 flimrer litt

Signal_1 = LOW  152	Signal_2 = HIGH  277	Signal_3 = HIGH  578	Signal_4 = HIGH  1004	Signal_5 = HIGH  1013
Signal_1 = LOW  144	Signal_2 = HIGH  348	Signal_3 = HIGH  744	Signal_4 = HIGH  1010	Signal_5 = HIGH  868
Signal_1 = LOW  133	Signal_2 = HIGH  295	Signal_3 = HIGH  615	Signal_4 = HIGH  965	Signal_5 = HIGH  993
Signal_1 = LOW  133	Signal_2 = HIGH  249	Signal_3 = HIGH  512	Signal_4 = HIGH  827	Signal_5 = HIGH  1007
Signal_1 = LOW  130	Signal_2 = HIGH  212	Signal_3 = HIGH  427	Signal_4 = HIGH  708	Signal_5 = HIGH  1015


På slutten og på starten:

Signal_1 = LOW  59	Signal_2 = LOW  59	Signal_3 = LOW  72	Signal_4 = LOW  82	Signal_5 = LOW  124
Signal_1 = LOW  59	Signal_2 = LOW  62	Signal_3 = LOW  73	Signal_4 = LOW  72	Signal_5 = LOW  125
Signal_1 = LOW  60	Signal_2 = LOW  65	Signal_3 = LOW  74	Signal_4 = LOW  76	Signal_5 = LOW  115
Signal_1 = LOW  54	Signal_2 = LOW  64	Signal_3 = LOW  75	Signal_4 = LOW  77	Signal_5 = LOW  130
