#ifndef TONETABLE_H
#define TONETABLE_H

/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | October 7th, 2019               |
 +---------------------------------*/

/* Data source: https://de.wikipedia.org/wiki/Frequenzen_der_gleichstufigen_Stimmung */

float tonetable[89] = {
/* 00 |                      */    20.0000
/* 01 | A0      | A2         */ ,  27.5000
/* 02 | A#0/Bb0 | Ais2/B2    */ ,  29.1352
/* 03 | B0      | H2         */ ,  30.8677
/* 04 | C1      | C1         */ ,  32.7032
/* 05 | C#1/Db1 | Cis1/Des1  */ ,  34.6478
/* 06 | D1      | D1         */ ,  36.7081
/* 07 | D#1/Eb1 | Dis1/Es1   */ ,  38.8909
/* 08 | E1      | E1         */ ,  41.2034
/* 09 | F1      | F1         */ ,  43.6535
/* 10 | F#1/Gb1 | Fis1/Ges1  */ ,  46.2493
/* 11 | G1      | G1         */ ,  48.9994
/* 12 | G#1/Ab1 | Gis1/As1   */ ,  51.9131
/* 13 | A1      | A1         */ ,  55.0000
/* 14 | A#1/Bb1 | Ais1/B1    */ ,  58.2705
/* 15 | B1      | H1         */ ,  61.7354
/* 16 | C2      | C          */ ,  65.4064
/* 17 | C#2/Db2 | Cis/Des    */ ,  69.2957
/* 18 | D2      | D          */ ,  73.4162
/* 19 | D#2/Eb2 | Dis/Es     */ ,  77.7817
/* 20 | E2      | E          */ ,  82.4069
/* 21 | F2      | F          */ ,  87.3071
/* 22 | F#2/Gb2 | Fis/Ges    */ ,  92.4986
/* 23 | G2      | G          */ ,  97.9989
/* 24 | G#2/Ab2 | Gis/As     */ , 103.826
/* 25 | A2      | A          */ , 110.000
/* 26 | A#2/Bb2 | Ais/B      */ , 116.541
/* 27 | B2      | H          */ , 123.471
/* 28 | C3      | c          */ , 130.813
/* 29 | C#3/Db3 | cis/des    */ , 138.591
/* 30 | D3      | d          */ , 146.832
/* 31 | D#3/Eb3 | dis/es     */ , 155.563
/* 32 | E3      | e          */ , 164.814
/* 33 | F3      | f          */ , 174.614
/* 34 | F#3/Gb3 | fis/ges    */ , 184.997
/* 35 | G3      | g          */ , 195.998
/* 36 | G#3/Ab3 | gis/as     */ , 207.652
/* 37 | A3      | a          */ , 220.000
/* 38 | A#3/Bb3 | ais/b      */ , 233.082
/* 39 | B3      | h          */ , 246.942
/* 40 | C4[3]   | c1         */ , 261.626
/* 41 | C#4/Db4 | cis1/des1  */ , 277.183
/* 42 | D4      | d1         */ , 293.665
/* 43 | D#4/Eb4 | dis1/es1   */ , 311.127
/* 44 | E4      | e1         */ , 329.628
/* 45 | F4      | f1         */ , 349.228
/* 46 | F#4/Gb4 | fis1/ges1  */ , 369.994
/* 47 | G4      | g1         */ , 391.995
/* 48 | G#4/Ab4 | gis1/as1   */ , 415.305
/* 49 | A4[2]   | a1         */ , 440.000
/* 50 | A#4/Bb4 | ais1/b1    */ , 466.164
/* 51 | B4      | h1         */ , 493.883
/* 52 | C5      | c2         */ , 523.251
/* 53 | C#5/Db5 | cis2/des2  */ , 554.365
/* 54 | D5      | d2         */ , 587.330
/* 55 | D#5/Eb5 | dis2/es2   */ , 622.254
/* 56 | E5      | e2         */ , 659.255
/* 57 | F5      | f2         */ , 698.456
/* 58 | F#5/Gb5 | fis2/ges2  */ , 739.989
/* 59 | G5      | g2         */ , 783.991
/* 60 | G#5/Ab5 | gis2/as2   */ , 830.609
/* 61 | A5      | a2         */ , 880.000
/* 62 | A#5/Bb5 | ais2/b2    */ , 932.328
/* 63 | B5      | h2         */ , 987.767
/* 64 | C6      | c3         */ ,1046.50
/* 65 | C#6/Db6 | cis3/des3  */ ,1108.73
/* 66 | D6      | d3         */ ,1174.66
/* 67 | D#6/Eb6 | dis3/es3   */ ,1244.51
/* 68 | E6      | e3         */ ,1318.51
/* 69 | F6      | f3         */ ,1396.91
/* 70 | F#6/Gb6 | fis3/ges3  */ ,1479.98
/* 71 | G6      | g3         */ ,1567.98
/* 72 | G#6/Ab6 | gis3/as3   */ ,1661.22
/* 73 | A6      | a3         */ ,1760.00
/* 74 | A#6/Bb6 | ais3/b3    */ ,1864.66
/* 75 | B6      | h3         */ ,1975.53
/* 76 | C7      | c4         */ ,2093.00
/* 77 | C#7/Db7 | cis4/des4  */ ,2217.46
/* 78 | D7      | d4         */ ,2349.32
/* 79 | D#7/Eb7 | dis4/es4   */ ,2489.02
/* 80 | E7      | e4         */ ,2637.02
/* 81 | F7      | f4         */ ,2793.83
/* 82 | F#7/Gb7 | fis4/ges4  */ ,2959.96
/* 83 | G7      | g4         */ ,3135.96
/* 84 | G#7/Ab7 | gis4/as4   */ ,3322.44
/* 85 | A7      | a4         */ ,3520.00
/* 86 | A#7/Bb7 | ais4/b4    */ ,3729.31
/* 87 | B7      | h4         */ ,3951.07
/* 88 | C8      | c5         */ ,4186.01
};

#endif /* TONETABLE_H */
