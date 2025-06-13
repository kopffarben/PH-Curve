# Zusammenfassung der Referenzen

Dieses Dokument fasst die Inhalte der PDF-Dateien im Ordner `References` zusammen. Zudem wird erläutert, wie sich die Ergebnisse in unserer Bibliothek **PH-Curve** widerspiegeln. Abschließend werden Erweiterungsmöglichkeiten und neue Funktionen skizziert.

## 1. Farouki & Saguin-Sprynski (2014)
Der Artikel **C2 interpolation of spatial data subject to arc-length constraints using Pythagorean–hodograph quintic splines** ([Lokale PDF](Farouki_Saguin_Sprynski_2014_C2_Interpolation.pdf)) entwickelt zwei PH-Quintik-Spline-Ansätze, mit denen räumliche Kurven rekonstruiert werden, wenn nur die Abstände zwischen Messrahmen bekannt sind. Aufgrund der polynomiellen Bogeneigenschaft von PH-Kurven lassen sich Längenbedingungen algebraisch formulieren. Das entstehende quadratische Optimierungsproblem wird mithilfe von Quaternionen gelöst.

## 2. Farouki et al. (2008)
In **Identification of spatial PH quintic Hermite interpolants with near-optimal shape measures** ([Lokale PDF](Farouki_et_al_2008_Identification_of_Spatial_PH_Quintic_Hermite_Interpolants.pdf)) werden Strategien vorgestellt, die freien Parameter einer PH-Quintik-Hermite-Interpolation so zu wählen, dass ein möglichst glatter Verlauf entsteht. Die Autoren zeigen, dass die Bogenlänge nur von einem Parameter abhängt, und stellen drei Verfahren zur Auswahl geeigneter Parameter vor. Dabei spielen quaternionische Darstellungen und helikale PH-Quintiken eine zentrale Rolle.

## 3. Jaklič et al. (2015)
Der Beitrag **Interpolation by G² quintic Pythagorean-hodograph curves** ([Lokale PDF](Jaklic_et_al_2015_G2_Quintic_PH_Interpolation.pdf)) behandelt die Konstruktion von G²-stetigen PH-Splines in beliebiger Dimension. Ausgehend von Punkten, Tangenten und Krümmungsvektoren wird das Problem auf zwei Polynomgleichungen für die Tangentenlängen reduziert. Ein Homotopie-Verfahren verfolgt anschließend die optimale Lösung, besonders bei mehreren möglichen Interpolanten.

## 4. Kozak (2014)
Die Arbeit **Global well-posedness of strong solutions to the 3D primitive equations with horizontal eddy diffusivity** ([Lokale PDF](Kozak_2014_Spatial_Rational_PH_Cubic.pdf)) untersucht partielle Differentialgleichungen der Geophysik. Sie steht in keinem direkten Bezug zu PH-Kurven, belegt jedoch die Existenz starker Lösungen für die primitiven Gleichungen bei horizontaler Diffusion.

## 5. Farouki & Dong (2012)
**PHquintic: A library of basic functions for the construction and analysis of planar quintic Pythagorean–hodograph curves** ([Lokale PDF](Farouki_Dong_2012_PHquintic_Library.pdf)) beschreibt eine umfangreiche Softwarebibliothek zur Erzeugung und Analyse planarer PH-Quintiken. Sie basiert auf der komplexen Darstellung, ermöglicht exakte Bogenlängen, Offsets und das Berechnen der Biegeenergie. Die Autoren betonen, dass Quintiken die niedrigste Ordnung darstellen, mit der beliebige Hermite-Daten interpoliert werden können.

## 6. Schröcker & Šír (2023)
**Optimal interpolation with spatial rational PH curves** ([Lokale PDF](Schroecker_Sir_2023_Optimal_Interpolation_with_Spatial_Rational_PH_Curves.pdf)) stellt eine Residuen-Methode vor, um rationale PH-Kurven zu konstruieren, die vorgegebene Tangentenrichtungen erfüllen. Dadurch lassen sich G1- und G2-Interpolationsprobleme linear formulieren und Optimierungen hinsichtlich Länge oder Biegeenergie durchführen.

## 7. Arrizabalaga & Ryll (2022)
**Spatial motion planning with Pythagorean Hodograph curves** ([Lokale PDF](Arrizabalaga_Ryll_2022_Spatial_Motion_Planning_with_PH_Curves.pdf)) präsentiert einen zweistufigen Ansatz, der Hindernisgeometrie in eine PH-Spline einbettet und anschließend einen kollisionsfreien Pfad im parametrisierten Freiraum sucht. Die kompakte PH-Darstellung erleichtert die Optimierung.

## 8. Arrizabalaga et al. (2024)
**PHODCOS: Pythagorean Hodograph-based Differentiable Coordinate System** ([Lokale PDF](Arrizabalaga_et_al_2024_PHODCOS.pdf)) beschreibt ein differenzierbares Koordinatensystem entlang einer Kurve, das mittels PH-Kurven exakte Ableitungen liefert und damit präzise robotische Navigation ermöglicht.

## Bezug zur Implementierung
Unsere Bibliothek setzt viele dieser Konzepte um:
- `PHCurve3D` stellt ein Quintik-Segment über die Hodograph-Koeffizienten A–E dar und liefert Position, Ableitungen, Geschwindigkeit und Bogenlänge.
- `HermiteControlPoint3D` speichert Position, Tangente, Krümmung und Hauptnormal entsprechend den Anforderungen aus den Publikationen.
- `PHCurveFactory` berechnet gemäß Jaklič et al. (2015) eine G²-Hermite-Interpolation und bietet eine Validierung der G²-Stetigkeit.
- `PathPlanner` fügt mehrere Segmente zusammen und prüft, ob Tangenten und Normalen an den Übergängen übereinstimmen.
- Die Testfälle orientieren sich an den Formeln aus Farouki & Dong (2012) und sichern die korrekte mathematische Umsetzung.

## Mögliche Erweiterungen
Aus den Referenzen lassen sich zahlreiche Erweiterungen ableiten:
1. **Spline-Anpassung mit Längenbindung** – Umsetzung der Optimierungsmethode von Farouki & Saguin-Sprynski (2014) zur Rekonstruktion von Sensordaten.
2. **Automatische Parameterwahl** – Integration der Kriterien aus Farouki et al. (2008), um die freien Parameter ohne manuelles Feintuning zu bestimmen.
3. **Homotopie-Löser** – Nutzung des in Jaklič et al. (2015) vorgestellten Homotopie-Verfahrens zur robusten Lösungsauswahl.
4. **Planare PHquintic-Bibliothek** – Einbindung der Funktionen aus Farouki & Dong (2012), um in 2D exakte Offsets und Energieauswertungen anzubieten.
5. **Rationale PH-Kurven** – Erweiterung auf rationale Varianten und rotationminimierende Frames.

## Neue Funktionsideen
Anhand der Literatur und unserer Umsetzung ergeben sich folgende mögliche Erweiterungen:
- **C²-PH-Spline-Fitting** mit optionalen Längenbedingungen zur Formrekonstruktion.
- **Modul zur Kurvenoptimierung**, das Biegeenergie, Bogenlänge und Krümmungsvariation auswertet.
- **Toolkit für Homotopie-Verfahren** zur Lösung komplexer Interpolationsprobleme.
- **Echtzeit-Trajektoriensteuerung** auf Basis der exakten Bogenlängenparametrisierung.
- **Unterstützung alternativer Übergangskurven** wie Klotoiden oder rationale Kubiken, falls PH-Kurven nicht geeignet sind.

Diese Ansätze können das Projekt erweitern und seine Einsatzmöglichkeiten in Robotik, CAD und Animation deutlich erhöhen.
