# Anleitung zur PH-Kurvenanpassung

Dieses Dokument beschreibt verschiedene Varianten zur Approximation von gemessenen Punkten durch PH-Kurven in `PathPlanner`. Jede Variante wird durch einen eigenen Agenten abgebildet. Zudem wird eine kontinuierliche Methode vorgestellt, um Kurven in Echtzeit zu erzeugen. Abschließend werden Tests skizziert, die alle Varianten abdecken. Zusätzlich enthält das Dokument einen kurzen Vergleich der Algorithmen mit Vor- und Nachteilen.

## Gemeinsame Vorgaben

- Eingabe ist stets eine `List<PointData>` mit `Vector3 Position`, `Vector3 UpVector` und `float Time`. Die Liste ist nach `Time` sortiert und kann Rauschen enthalten.
- Ausgabe ist eine `List<PHCurve3D>`; jedes Segment trägt zusätzlich die absolute Start- und Endzeit.
- `positionTolerance` und `orientationTolerance` geben an, wie stark die approximierte Kurve vom Messpfad abweichen darf.
- Up-Vektoren sollen möglichst in Richtung der Normalen zeigen, sind aber nachrangig gegenüber Position und Tangente.
- Nach jedem neuen Segment erfolgt eine G²-Optimierung mit dem vorherigen Segment.
- Implementiere in `PathPlanner` die Methode `CurveFitting(List<PointData> points, float positionTolerance, float orientationTolerance)`. Sie soll die Eingabepunkte analysieren, eine minimale Anzahl an `PHCurve3D`-Segmenten erzeugen und dabei den zeitlichen Verlauf respektieren.

Zur mathematischen Hintergrundinformation verweisen die einzelnen Varianten auf lokale PDF-Dateien im Ordner `References`.

## 1. Lokale Hermite-Interpolation

- **Literatur:** [Jaklic et al. 2015](PHCurveLibrary/References/Jaklic_et_al_2015_G2_Quintic_PH_Interpolation.pdf)
- **Prinzip:** Zwischen jeweils zwei Punkten wird eine quintische PH-Kurve erzeugt, die Position, Tangente und Krümmung (abgeleitet aus den benachbarten Punkten) erfüllt.
- **Vorteile:**
  - Sehr schnelle Berechnung, da nur lokale Gleichungssysteme gelöst werden.
  - G²-Stetigkeit kann direkt über die Enddaten erreicht werden.
- **Nachteile:**
  - Keine globale Fehlerverteilung; Rauschen führt zu vielen kurzen Segmenten.
- **Rechenaufwand:** Pro Segment konstante Zeit, Gesamtlaufzeit linear in der Segmentanzahl.
- **Agent-Prompt:**
  1. Erstelle `FitLocalHermite(List<PointData> points, float positionTolerance, float orientationTolerance)`.
  2. Berechne aus aufeinanderfolgenden Punkten die benötigten Hermite-Daten und rufe `PHCurveFactory.CreateQuintic` auf.
  3. Führe eine kurze Glättung durch, falls der Up-Vektor zu stark variiert.
  4. Prüfe, ob der Fehler unter den Toleranzen liegt, sonst Segmentanzahl erhöhen.
- **Kontinuierliche Methode:**
  - `FitLocalHermiteIncremental(List<PointData> buffer, ...)` liest jedes Frame neue Punkte ein, erstellt solange Segmente, bis die Toleranz überschritten würde, leert dann den Puffer und setzt fort.

## 2. Globale Least-Squares-Approximation

- **Literatur:** [Farouki, Saitou, Tsai 1998](PHCurveLibrary/References/Fitting/Farouki_Saitou_Tsai_1998_Least_Squares_Approximation_PH_Curvs.pdf)
- **Prinzip:** Bestimme PH-Kurvenparameter durch Minimierung des quadratischen Fehlers über alle Punkte eines Segments. Adaptive Unterteilung bei zu großem Fehler.
- **Vorteile:**
  - Sehr genaue Anpassung an den gesamten Punktverlauf.
  - Ermöglicht eine kontrollierte Segmentanzahl.
- **Nachteile:**
  - Benötigt nichtlineare Optimierung und kann lokale Minima besitzen.
  - Hoher Rechenaufwand, stark abhängig von der Anzahl der Iterationen.
- **Rechenaufwand:** In der Größenordnung \(O(k n)\) bis \(O(k n^2)\), wobei \(k\) die Iterationszahl und \(n\) die Punktzahl ist.
- **Agent-Prompt:**
  1. Implementiere `FitLeastSquares(List<PointData> points, float positionTolerance, float orientationTolerance)`.
  2. Verwende Newton-Raphson, um die Hodographkoeffizienten zu optimieren. Nutze Up-Vektor-Abweichungen als weichen Strafterm.
  3. Unterteile das Intervall, falls der Fehler oberhalb der Toleranz bleibt.
- **Kontinuierliche Methode:**
  - `FitLeastSquaresIncremental(...)` führt die Optimierung immer nur über den aktuellen Puffer aus und erzeugt ein oder mehrere Segmente, sobald die Toleranz erreicht ist.

## 3. Evolutionsbasierte Anpassung

- **Literatur:** [Aigner, Jüttler et al. 2007](PHCurveLibrary/References/Fitting/Aigner_Sir_Jüttler_2007_Least_Squares_Fitting.pdf)
- **Prinzip:** Eine Startkurve wird über eine Evolutionsgleichung iterativ an die Daten angepasst. Nebenbedingungen für Glätte und Up-Vektor werden durch Energieterme berücksichtigt.
- **Vorteile:**
  - Robust gegenüber Ausreißern und ungenauer Parametrisierung.
  - Flexibel kombinierbar mit weiteren Glättungs- oder Energiebegriffen.
- **Nachteile:**
  - Lange Rechenzeit durch viele Evolutionsschritte.
  - Parameterwahl (Schrittweite, Gewichtung) nicht trivial.
- **Rechenaufwand:** Häufig \(O(k n)\) oder höher, wobei \(k\) die Anzahl der Evolutionsschritte und \(n\) die Punktzahl ist.
- **Agent-Prompt:**
  1. `FitEvolution(List<PointData> points, float positionTolerance, float orientationTolerance)` erzeugt eine initiale PH-Kurve und minimiert dann eine Energie \( E = \alpha E_{pos} + \beta E_{orient} + \gamma E_{smooth} \).
  2. Stoppe die Evolution, sobald alle Toleranzen erfüllt sind oder eine Maximalanzahl an Iterationen erreicht wird.
- **Kontinuierliche Methode:**
  - `FitEvolutionIncremental(...)` führt pro Frame einen Evolutionsschritt auf dem aktuellen Puffer durch und materialisiert Segmente, wenn die Toleranz eingehalten wird.

## 4. Homotopie-Fortsetzung

- **Literatur:** [Albrecht & Farouki 1996](PHCurveLibrary/References/Schroecker_Sir_2023_Optimal_Interpolation_with_Spatial_Rational_PH_Curves.pdf)
- **Prinzip:** Starte von einer einfach berechnbaren Kurve und deformiere sie über einen Homotopieparameter schrittweise zur gewünschten Lösung. Dabei bleiben G²-Bedingungen strikt erhalten.
- **Vorteile:**
  - Vermeidet abrupte Änderungen und kann mehrere gültige Lösungen verfolgen.
  - Gut geeignet bei stark nichtlinearen Randbedingungen.
- **Nachteile:**
  - Implementierung komplex und abhängig von geeigneter Schrittweitenwahl.
  - Rechenaufwand schwankt je nach Verlauf des Homotopiepfades.
- **Rechenaufwand:** Typischerweise \(O(k n)\) bis \(O(k n^2)\), abhängig von der Anzahl der Fortsetzungsschritte.
- **Agent-Prompt:**
  1. `FitHomotopy(List<PointData> points, float positionTolerance, float orientationTolerance)` initialisiert mit einer linearen oder kreisförmigen Kurve.
  2. Verfolge den Lösungsweg mithilfe eines Prädiktor-Korrektor-Verfahrens, bis alle Punkte innerhalb der Toleranzen liegen.
  3. Verfeinere die Zeitparametrisierung nach Bedarf.
- **Kontinuierliche Methode:**
  - `FitHomotopyIncremental(...)` nutzt den zuvor berechneten Homotopiepfad als Startwert für das nächste Segment und erzeugt in Echtzeit neue Segmente.

## 5. Heuristische Segmentierung mit anschließender Glättung

- **Literatur:** Kombination klassischer Kurvensegmentierungsansätze, siehe [Kosinka & Lavicka 2014](PHCurveLibrary/References/Fitting/Kosinka_Lavicka_2014_Survey_of_Recent_Advances.pdf)
- **Prinzip:** Die Punktliste wird zunächst grob in Teilabschnitte unterteilt, etwa anhand von Abstand oder Tangentenänderung. Innerhalb jedes Abschnitts wird eine der oben genannten Methoden angewendet und die Übergänge anschließend geglättet.
- **Vorteile:**
  - Geringerer Optimierungsaufwand, da nur Teilstücke angepasst werden.
  - Ermöglicht anpassbare Balance zwischen Laufzeit und Genauigkeit.
- **Nachteile:**
  - Qualität der Ergebnisse hängt stark von der Segmentierungsheuristik ab.
  - G²-Kontinuität muss nachträglich geprüft und korrigiert werden.
- **Rechenaufwand:** Im Idealfall linear bis leicht überlinear in der Punktzahl, abhängig von der gewählten Heuristik.
- **Agent-Prompt:**
  1. `FitHeuristic(List<PointData> points, ...)` analysiert die Eingabe, bestimmt geeignete Segmentgrenzen und ruft anschließend eine der Varianten 1–4 auf.
  2. Die Übergangsbereiche werden mit `PHCurveFactory.ValidateG2` kontrolliert und bei Bedarf nachgebessert.
- **Kontinuierliche Methode:**
- `FitHeuristicIncremental(...)` überwacht fortlaufend den eingehenden Punktepuffer, markiert heuristisch einen Abschnitt als abgeschlossen und glättet anschließend die Übergänge zu bereits existierenden Segmenten.

## Algorithmusvergleich

Die fünf Varianten unterscheiden sich im Aufwand und der erzielbaren Qualität:

| Methode | Vorteile | Nachteile | Rechenaufwand |
|---------|----------|-----------|---------------|
| Lokale Hermite | Schnell, einfache Implementierung | Viele Segmente bei Rauschen | Linear in der Segmentanzahl |
| Least Squares | Sehr genaue Anpassung, kontrollierbare Segmentzahl | Nichtlineare Optimierung, Gefahr lokaler Minima | \(O(k n)\) bis \(O(k n^2)\) |
| Evolution | Robust und flexibel | Lange Rechenzeit, Parameterwahl schwierig | meist \(O(k n)\) oder höher |
| Homotopie | Verfolgt kontinuierliche Lösungen | Komplexe Implementierung, variabler Aufwand | \(O(k n)\) bis \(O(k n^2)\) |
| Heuristische Segmentierung | Reduzierter Optimierungsaufwand | Ergebnisqualität abhängig von Heuristik | nah an linear |

## Gemeinsame Tests

Alle Varianten benötigen umfangreiche Unit-Tests im Projekt `PHCurveLibrary.Test`:

1. **Lange Eingabesequenzen:** Erzeuge synthetische Punktfolgen mit mehr als hundert Einträgen und überprüfe, dass die Anzahl der erzeugten Segmente minimal bleibt.
2. **Up-Vektor-Konsistenz:** Prüfe an Stichpunkten entlang jedes Segments, ob der Winkel zwischen berechnetem Normalenvektor und Up-Vektor einen Schwellwert nicht überschreitet.
3. **Toleranzvarianten:** Führe die Algorithmen mit mehreren Position- und Orientierungs-Toleranzen aus (z.B. 0.1, 0.01, 0.001) und verifiziere, dass die Fehler entsprechend skalieren.
4. **G²-Kontinuität:** Kontrolliere mit `PHCurveFactory.ValidateG2`, dass aufeinanderfolgende Segmente innerhalb des Toleranzbereichs liegen.
5. **Zeitaspekt (Grenzen):** Teste, dass die absoluten Start- und Endzeiten der Segmente exakt den Eingabepunkten entsprechen und monoton ansteigen.
6. **Zeitaspekt (Zwischenwerte):** Für mehrere frei wählbare Zeitpunkte innerhalb eines Segments soll die Position sowie der Up-Vektor mit den Eingangsdaten übereinstimmen. Errechne dazu die Abweichung zwischen Kurve und Originalpunkten und überprüfe, dass sie unter `positionTolerance` bzw. `orientationTolerance` liegt.

Die Tests sollen aussagekräftige Konsolenausgaben liefern und gemäß [AGENTS.md](AGENTS.md) in englischer Sprache dokumentiert werden.

## Benchmarks

Zum Vergleich der Verfahren sollen im Testprojekt auch einfache Benchmarks entstehen. Hier bietet sich [BenchmarkDotNet](https://benchmarkdotnet.org/) an.

- Messe die Ausführungszeit der Methoden `FitLocalHermite`, `FitLeastSquares`, `FitEvolution`, `FitHomotopy` sowie ihrer inkrementellen Varianten bei identischen Punktfolgen (mindestens tausend Punkte).
- Halte zusätzlich die Anzahl der erzeugten Segmente und den maximalen Fehler fest.
- Fasse die Ergebnisse in einer Tabelle zusammen, um Geschwindigkeits- und Qualitätsunterschiede sichtbar zu machen.

## Integration der absoluten Zeit

Erweitere `PHCurve3D` um `float StartTime` und `float EndTime`. Konstruktoren und Fabrikmethoden müssen diese Werte übernehmen. Die Zeitinformation erlaubt, jede Kurve in Echtzeit abzuspielen und an das Sensor-Frame anzubinden.

## Zusammenfassung

Dieses Dokument bildet die Grundlage für die Implementierung verschiedener PH-Kurvenanpassungen samt Echtzeitvariante. Die angegebenen Agent-Prompts und Tests helfen dabei, die Algorithmen konsistent in das bestehende Projekt einzufügen.
