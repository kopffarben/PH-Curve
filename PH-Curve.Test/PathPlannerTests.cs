using Microsoft.VisualStudio.TestTools.UnitTesting;
using System.Numerics;
using PHCurveLibrary;

namespace PHCurveLibrary.Tests
{
    [TestClass]
    public class PathPlannerTests
    {
        // Diese Tests überprüfen den PathPlanner gemäß den in der
        // Dokumentation (siehe AGENTS.md) aufgeführten Methoden zur
        // G^2-Interpolation von PH-Kurven nach Jaklič et al. (2015).
        // Diese Hilfsmethode erzeugt einen Hermite-Punkt ohne vorgegebene
        // Krümmung. Sie verkürzt die Testfälle, in denen nur Position und
        // Tangentenrichtung relevant sind.
        private static HermiteControlPoint3D CreatePoint(Vector3 pos, Vector3 tan)
        {
            return new HermiteControlPoint3D(pos, tan, 0f, Vector3.UnitY);
        }

        // Überladene Variante von <see cref="CreatePoint(Vector3,Vector3)"/>,
        // bei der zusätzlich Krümmung und Hauptnormal angegeben werden können.
        private static HermiteControlPoint3D CreatePoint(Vector3 pos, Vector3 tan, float k, Vector3 n)
        {
            return new HermiteControlPoint3D(pos, tan, k, n);
        }

        // Berechnet die Krümmung eines erzeugten PH-Segments. Laut Definition ist
        // \(\kappa = \|r'(t) \times r''(t)\|/\|r'(t)\|^3\). Diese Formel findet
        // sich u. a. bei Farouki (2014) und wird hier verwendet, um die vom
        // PathPlanner gelieferten Werte mit den vorgegebenen Daten zu vergleichen.
        private static float Curvature(PHCurve3D c, float t)
        {
            Vector3 d1 = c.Derivative(t);
            Vector3 d2 = c.SecondDerivative(t);
            Vector3 cross = Vector3.Cross(d1, d2);
            return cross.Length() / MathF.Pow(d1.Length(), 3f);
        }

        [TestMethod]
        public void BuildPath_ReturnsAllSegments()
        {
            // Dieser Test prüft allein, ob die interne Segmentliste im Planner
            // korrekt ausgegeben wird. Die gewählten Punkte sind identisch und
            // erzeugen zwar degenerierte Kurven, beeinflussen aber nicht die
            // Erwartungshaltung hinsichtlich der Anzahl der Segmente.
            var planner = new PathPlanner();
            var p0 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p1 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p2 = CreatePoint(Vector3.Zero, Vector3.UnitX);

            planner.AddSegment(p0, p1);
            planner.AddSegment(p1, p2);
            var path = planner.BuildPath();

            // Erwartet werden zwei Segmente, da wir zweimal AddSegment aufgerufen haben.
            Assert.AreEqual(2, path.Count);
        }

        [TestMethod]
        public void ValidatePathG2_ReturnsTrueForContinuousPath()
        {
            // Bei identischen Punkten und Tangenten sollte G^2-Stetigkeit trivial erfüllt sein.
            var planner = new PathPlanner();
            var p0 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p1 = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var p2 = CreatePoint(Vector3.Zero, Vector3.UnitX);

            planner.AddSegment(p0, p1);
            planner.AddSegment(p1, p2);

            // ValidatePathG2 benutzt PHCurveFactory.ValidateG2 nach Jaklič et al. (2015).
            Assert.IsTrue(planner.ValidatePathG2());
        }

        [TestMethod]
        public void ValidatePathG2_ReturnsFalseForDiscontinuousPath()
        {
            // Hier unterscheiden sich Tangente und Normalenrichtung an der
            // Verbindungsstelle. Damit verletzen wir bewusst die G^2-Bedingung.
            var planner = new PathPlanner();
            var start = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var junctionEnd = CreatePoint(Vector3.Zero, Vector3.UnitX);
            var junctionStart = CreatePoint(Vector3.Zero, Vector3.UnitY);
            var end = CreatePoint(Vector3.Zero, Vector3.UnitY);

            planner.AddSegment(start, junctionEnd);
            planner.AddSegment(junctionStart, end);

            Assert.IsFalse(planner.ValidatePathG2());
        }

        [TestMethod]
        public void BuildPath_WithStraightLinePoints()
        {
            // Drei auf einer Geraden liegende Punkte. Die Tangenten zeigen alle
            // in x-Richtung. Damit sollte jedes Segment eine reine
            // Translation ohne Krümmung ergeben.
            var planner = new PathPlanner();
            var points = new[]
            {
                CreatePoint(new Vector3(0f, 0f, 0f), Vector3.UnitX),
                CreatePoint(new Vector3(1f, 0f, 0f), Vector3.UnitX),
                CreatePoint(new Vector3(2f, 0f, 0f), Vector3.UnitX)
            };

            for (int i = 0; i < points.Length - 1; i++)
            {
                planner.AddSegment(points[i], points[i + 1]);
            }

            var path = planner.BuildPath();
            // Die Anzahl der erzeugten Segmente muss der Punktzahl minus eins entsprechen.
            Assert.AreEqual(points.Length - 1, path.Count);
        }

        [TestMethod]
        public void BuildPath_WithSemicirclePoints()
        {
            // Punkte und Tangenten liegen auf einem Halbkreis mit Radius eins.
            // Dieser Test stellt sicher, dass der Planner auch bei gekrümmten
            // Trajektorien konsistente Segmente erzeugt.
            var planner = new PathPlanner();
            var points = new[]
            {
                CreatePoint(new Vector3(1f, 0f, 0f), new Vector3(0f, 1f, 0f)),
                CreatePoint(new Vector3(0f, 1f, 0f), new Vector3(-1f, 0f, 0f)),
                CreatePoint(new Vector3(-1f, 0f, 0f), new Vector3(0f, -1f, 0f))
            };

            for (int i = 0; i < points.Length - 1; i++)
            {
                planner.AddSegment(points[i], points[i + 1]);
            }

            var path = planner.BuildPath();
            Assert.AreEqual(points.Length - 1, path.Count);
        }

        [TestMethod]
        public void BuildPath_WithSinePoints()
        {
            // Hier approximieren wir einen Sinusabschnitt. Die gewählten Punkte
            // liegen auf der Funktion y = sin(x) für x \in [0, \pi]. Die
            // Tangenten entsprechen den Ableitungen der Sinusfunktion. Der Test
            // dient vor allem als Beispiel für wechselnde Krümungsrichtungen.
            var planner = new PathPlanner();
            var points = new[]
            {
                CreatePoint(new Vector3(0f, 0f, 0f), new Vector3(1f, 1f, 0f)),
                CreatePoint(new Vector3(MathF.PI / 2f, 1f, 0f), new Vector3(1f, 0f, 0f)),
                CreatePoint(new Vector3(MathF.PI, 0f, 0f), new Vector3(1f, -1f, 0f))
            };

            for (int i = 0; i < points.Length - 1; i++)
            {
                planner.AddSegment(points[i], points[i + 1]);
            }

            var path = planner.BuildPath();
            Assert.AreEqual(points.Length - 1, path.Count);
        }

        [TestMethod]
        public void BuildPath_PreservesCurvatureAndPrincipalNormals()
        {
            // Anhand zweier Hermite-Punkte wird überprüft, ob der erzeugte
            // PH-Spline die vorgegebenen Krümmungen und Hauptnormalen an den
            // Enden exakt übernimmt. Dies basiert auf den Gleichungen zur G^2-
            // Interpolation aus Jaklič et al. (2015).
            var planner = new PathPlanner();
            var start = CreatePoint(new Vector3(0f, 0f, 0f), Vector3.UnitX, 0.1f, Vector3.UnitY);
            var end = CreatePoint(new Vector3(1f, 1f, 0f), Vector3.UnitY, 0.2f, -Vector3.UnitX);

            planner.AddSegment(start, end);
            var path = planner.BuildPath();
            Assert.AreEqual(1, path.Count);

            var seg = path[0];
            // Vergleiche Krümmung an den Segmentenden. Die Toleranz orientiert
            // sich an den numerischen Beispielen der PHquintic Library
            // (Farouki & Dong, 2012).
            Assert.AreEqual(start.Curvature, Curvature(seg, 0f), 1e-3f);
            Assert.AreEqual(end.Curvature, Curvature(seg, 1f), 1e-3f);

            // Zusätzlich müssen die Hauptnormalen übereinstimmen.
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(start.PrincipalNormal), seg.PrincipalNormal(0f)) < 1e-3f);
            Assert.IsTrue(Vector3.Distance(Vector3.Normalize(end.PrincipalNormal), seg.PrincipalNormal(1f)) < 1e-3f);
        }

        [TestMethod]
        public void ValidatePathG2_FailsWhenNormalsDiffer()
        {
            // An der Verbindungsstelle wird absichtlich eine andere Normalen-
            // richtung vorgegeben. Laut Definition muss G^2-Kontinuität dann
            // verletzt sein.
            var planner = new PathPlanner();
            var start = CreatePoint(Vector3.Zero, Vector3.UnitX, 0.1f, Vector3.UnitY);
            var jointEnd = CreatePoint(Vector3.One, Vector3.UnitX, 0.1f, Vector3.UnitY);
            var jointStart = CreatePoint(Vector3.One, Vector3.UnitX, 0.1f, Vector3.UnitZ);
            var end = CreatePoint(new Vector3(2f, 1f, 0f), Vector3.UnitY, 0.2f, -Vector3.UnitX);

            planner.AddSegment(start, jointEnd);
            planner.AddSegment(jointStart, end);

            // Die G^2-Prüfung muss fehlschlagen, da die Normalen am Gelenk
            // nicht zusammenpassen.
            Assert.IsFalse(planner.ValidatePathG2());
        }
    }
}
