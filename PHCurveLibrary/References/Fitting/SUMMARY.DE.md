# Bézier-Kurvenanpassung mit Pythagorean Hodograph (PH)-Kurven

## Einleitung
Die Kurvenanpassung mit Bézier- und Spline-Kurven ist ein zentrales Problem in der Computergrafik, im CAD/CAM und in der Robotik. Klassische polynomiale oder splinebasierte Methoden bieten oft keine geschlossenen Lösungen für wichtige geometrische Größen wie Bogenlänge und Krümmung. **Pythagorean Hodograph (PH)**-Kurven, eine spezielle Klasse polynomialer Kurven, umgehen diese Einschränkungen, indem sie geschlossene Formeln für die Bogenlänge und rationale Offsets ermöglichen.

Dieses Dokument gibt einen umfassenden Überblick über die mathematischen Grundlagen, zentrale Algorithmen und Veröffentlichungen zur PH-Kurvenanpassung, mit Schwerpunkt auf Anwendungen in der Computergrafik. Es werden zudem Anwendungen im CAD, CNC und der Robotik berücksichtigt. Die Mathematik wird im Detail erklärt, Algorithmen verglichen sowie Leistung und Nutzbarkeit bewertet. Wo möglich, sind kostenlose PDF-Links enthalten.

## Mathematische Grundlagen von PH-Kurven

### Klassische Bézier-Kurven
Eine planare Bézier-Kurve vom Grad \( n \) wird typischerweise dargestellt als:
\[
P(t) = \sum_{i=0}^{n} B_i^n(t) P_i,
\]
wobei \( B_i^n(t) \) die Bernstein-Polynome und \( P_i \) die Kontrollpunkte sind.

### Definition von PH-Kurven
Eine polynomiale Kurve \( P(t) = (x(t), y(t)) \) ist eine **Pythagorean Hodograph**-Kurve, wenn gilt:
\[
x'(t)^2 + y'(t)^2 = \sigma(t)^2,
\]
wobei \( \sigma(t) \) ein Polynom ist. Dies ermöglicht:
- Geschlossene Bogenlänge: \( s(t) = \int_0^t \sigma(u) \, du \)
- Rationale Offset-Kurven
- Einfache Geschwindigkeitsprofile und Interpolation

Der zentrale Gedanke: \( x'(t) + i y'(t) = [f(t) + i g(t)]^2 \) mit reellen Polynomen \( f(t), g(t) \). Dann ergibt sich:
\[
x'(t)^2 + y'(t)^2 = (f(t)^2 + g(t)^2)^2.
\]

### Praktische Vorteile
- Exakte Bogenlängen-Parametrisierung
- Rationale Offset-Kurven (wichtig für Werkzeugbahnen)
- Stetige Krümmung (\( G^2 \))
- Analytische Umkehrung der Zeit-Parametrisierung (\( s(t) \rightarrow t(s) \))

Diese Eigenschaften machen PH-Kurven ideal für CNC-Interpolation, Bewegungssteuerung und Pfadglättung in Grafik und Robotik.

## Zentrale Algorithmen und Veröffentlichungen

### 1. Lokale Hermite-Interpolation (Meek & Walton, Farouki et al.)
- Konstruktion stückweiser PH-Segmente (oft Quintiken)
- Hermite-Daten: Endpunkte, Tangentenrichtung, Krümmung
- Lösung nichtlinearer Systeme pro Segment mittels Newton-Verfahren
- Für interaktive und lokale Bearbeitung geeignet

### 2. Globale Least-Squares-Approximation (Farouki, Saitou, Tsai 1998)
- Minimierung des quadratischen Fehlers zwischen Daten und PH-Kurve
- Newton-Raphson oder Simulated Annealing
- Adaptive Segmentierung: Unterteilen bei zu großem Fehler
- Fußpunkt-Verbesserung: Parameterzuweisung iterativ optimieren
- Strafterme: z. B. Biegeenergie, Rotationsmaß
- PDF-Link: [Kostenloser Download](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.38.7640&rep=rep1&type=pdf) [Local PDF](Farouki_Saitou_Tsai_1998_Least_Squares_Approximation_PH_Curvs.pdf)

### 3. Evolutionsbasierte Anpassung (Aigner, Jüttler et al. 2007)
- Dynamische Evolution: \( P(t, \tau) \) entwickelt sich über fiktive Zeit \( \tau \)
- Inspiriert von aktiven Konturen („Snakes“)
- Konvergenz ähnlich Gauss-Newton-Verfahren
- Robust gegenüber Parametrisierungsproblemen
- PDF-Link: [Kostenloser Download](https://www.ag.jku.at/ftp/pub/Preprints/JKU-2007-02.pdf) [Local PDF](Aigner_Sir_Jüttler_2007_Least_Squares_Fitting.pdf)

### 4. Homotopie-Fortsetzung (Albrecht & Farouki 1996)
- Verbindung einer einfachen Ausgangskurve mit Zielkurve
- Sicherstellung \( C^2 \)-Stetigkeit zwischen PH-Segmenten
- Lösung komplexer Interpolationsprobleme auf globaler Ebene
- PDF-Link: [Local PDF](Fitting/Farouki_Albrecht_1996_C2_PH%20Curve_homotopy_methode.pdf)

### 5. Theorie und neuere Entwicklungen
- **Weierstraß-Theorem** für PH-Kurven (Choi & Moon 2008): beliebig gute Approximation
 - **Rationale PH-Kurven** und PH-Kurven in Minkowski-Räumen (Kosinka & Lavička 2014) [Local PDF](Kosinka_Lavicka_2014_Survey_of_Recent_Advances.pdf)
- **Robotik**: Pfade mit Krümmungs- und Steuergrenzwerten
 - **Hochgeschwindigkeits-CNC**: vibrationsfreie Bewegung und konstante Abtragsraten [Local PDF](Farouki_Computer_numerical_control_algorithms.pdf)

## Leistungsbewertung

| Methode                   | Typ        | Genauigkeit  | Geschwindigkeit  | Vorteile                                           | Nachteile                                     |
|--------------------------|------------|--------------|------------------|----------------------------------------------------|-----------------------------------------------|
| Hermite-Interpolation    | Lokal      | Mittel       | Schnell          | Einfach umsetzbar, interaktiv                     | Keine globale Fehlerkontrolle                |
| Least-Squares (Farouki)  | Global     | Hoch         | Mittel           | Hohe Präzision, adaptiv, analytische Bogenlänge   | Nichtlineare Gleichungen, lokale Minima      |
| Evolution (Jüttler)      | Global     | Hoch         | Langsam (iterativ)| Robust, flexibel                                  | Rechenintensiv                               |
| Homotopie (Albrecht)     | Global     | Hoch         | Variabel         | Löst komplexe Probleme über stetige Transformation| Aufwendig in der Umsetzung                   |

## Anwendungsbereiche

- **Computergrafik**: Faire Kurvengestaltung, Offsetsteuerung, Bogenlängen-Parametrisierung
- **CNC und CAD/CAM**: Echtzeitinterpolatoren, vibrationsfreie Hochgeschwindigkeitsbearbeitung
- **Robotik und Pfadplanung**: Krümmungsbegrenzte, zeitsynchrone Trajektorien

## Fazit
PH-Kurven bieten eine bemerkenswerte Kombination aus geometrischer Flexibilität und analytischer Handhabbarkeit. Ihre Fähigkeit, Bogenlängen und Offsets exakt zu berechnen und glatte Krümmungsverläufe zu ermöglichen, macht sie besonders geeignet für hochpräzise Anwendungen in Raum und Zeit.

Trotz der höheren Komplexität im Vergleich zu klassischen Splines haben sich PH-Kurvenmethoden als praktische Werkzeuge in der Technik und Grafik etabliert. Die Algorithmen reichen von schnellen lokalen Interpolationen bis zu robusten globalen Optimierern. Richtig implementiert, ermöglichen sie eine deutlich verbesserte Pfadqualität und Effizienz.

## Empfohlene Literatur und Downloads
 - Farouki et al. 1998: [Least-Squares-Approximation mit PH-Kurven (PDF)](https://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.38.7640&rep=rep1&type=pdf) [Local PDF](Farouki_Saitou_Tsai_1998_Least_Squares_Approximation_PH_Curvs.pdf)
 - Aigner et al. 2007: [Evolutionsbasierte PH-Anpassung (PDF)](https://www.ag.jku.at/ftp/pub/Preprints/JKU-2007-02.pdf) [Local PDF](Aigner_Sir_Jüttler_2007_Least_Squares_Fitting.pdf)
 - Kosinka & Lavička 2014: *PH-Kurven-Übersicht* (Open Access) [Local PDF](Kosinka_Lavicka_2014_Survey_of_Recent_Advances.pdf)
 - Pastva 1998: *Bezier Curve Fitting* [Local PDF](Pastva_1998_Bezier_Curve_Fitting.pdf)
 - Farouki: *Computer Numerical Control Algorithms* [Local PDF](Farouki_Computer_numerical_control_algorithms.pdf)
- Farouki 2008: *Pythagorean-Hodograph Curves: Algebra and Geometry Inseparable* (Buch)
- Albrecht & Farouki 1996: *C²-PH-Splines per Homotopie* [Local PDF](Fitting/Farouki_Albrecht_1996_C2_PH%20Curve_homotopy_methode.pdf)

Gib mir Bescheid, wenn du eine Implementierung, Grafiken oder eine LaTeX/PDF-Version benötigst.

