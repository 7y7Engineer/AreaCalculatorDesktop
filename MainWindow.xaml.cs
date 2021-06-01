using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;

namespace AreaCalculatorDesktop
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            new AreaCalculator();
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public class AreaCalculator
    {
        public AreaCalculator()
        {
            Polygon convex = new Convex(new List<Point>() { new Point(0, 0), new Point(0, 10), new Point(10, 10), new Point(10, 0) });
            var vertices = convex.Vertexes;
            var line1 = new LinearObject(1, 5, 5, -7); // 5x\ +\ 5y\ -\ 7\ =0
            var line2 = new LinearObject(2, 2, 5, -6); // 2x\ +\ 5y\ -\ 6\ =0

            var points1 = CollisionLineAndPolygon(vertices, line1.LinearEquationInstance.CalculatePoint(-100),
                line1.LinearEquationInstance.CalculatePoint(100));

            var points2 = CollisionLineAndPolygon(vertices, line2.LinearEquationInstance.CalculatePoint(-100),
                line2.LinearEquationInstance.CalculatePoint(100));

            var squareLineCrossPolygon1 = LengthSection(points1[0], points1[1]) * line1.Width;

            var squareLineCrossPolygon2 = LengthSection(points2[0], points2[1]) * line2.Width;

            var sum = squareLineCrossPolygon1 + squareLineCrossPolygon2 - line1.Width * line2.Width;
            /*var linearsObjects = new List<LinearObject>();
            linearsObjects.Add(line1);
            linearsObjects.Add(line2);*/

            /*var points1 = CollisionLineAndPolygon(vertexes, new Point(5, -100), new Point(5, 100)); // x = 5
            var points2 = CollisionLineAndPolygon(vertexes, new Point(-100, -100), new Point(100, 100));  // y = x
            var points3 = CollisionLineAndPolygon(vertexes, new Point(-50, -100), new Point(50, 100));  // y = 2x
            var points4 = CollisionLineAndPolygon(vertexes, new Point(0, -100), new Point(0, 100));  // x = 0
            var points5 = CollisionLineAndPolygon(vertexes, new Point(-100, 0), new Point(100, 0));  // y = 0*/
        }

        /// <summary>
        /// Метод, проверяющий пересекаются ли 2 отрезка [p1, p2] и [p3, p4]
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        /// <param name="p4"></param>
        /// <returns> 
        /// Return null if lines isn't crossing
        /// Return point of crossing, if lines is crossing
        /// </returns>
        public Point? CrossingLines(Point p1, Point p2, Point p3, Point p4)
        {
            double Xa, Ya;
            double A1, b1;
            double A2, b2;

            // p1.X <= p2.X
            if (p2.X < p1.X)
            {
                Point tmp = p1;
                p1 = p2;
                p2 = tmp;
            }
            // p3.X <= p4.X
            if (p4.X < p3.X)
            {
                Point tmp = p3;
                p3 = p4;
                p4 = tmp;
            }
            // Check interval
            if (p2.X < p3.X)
            {
                return null;
            }
            // If both segments are vertical
            if ((p1.X - p2.X == 0) && (p3.X - p4.X == 0))
            {
                // If they lie on the same X
                if (p1.X == p3.X)
                {
                    // Check if they intersect, if they have a common Y
                    if (!((Math.Max(p1.Y, p2.Y) < Math.Min(p3.Y, p4.Y)) || (Math.Min(p1.Y, p2.Y) > Math.Max(p3.Y, p4.Y))))
                    {
                        // The segments have no mutual abscissa
                        return null;
                    }
                }
                return null;
            }
            // If the first segment is vertical
            if (p1.X - p2.X == 0)
            {
                // Find Xa, Ya - intersection points of two lines
                Xa = p1.X;
                A2 = (p3.Y - p4.Y) / (p3.X - p4.X);
                b2 = p3.Y - A2 * p3.X;
                Ya = A2 * Xa + b2;
                if (p3.X <= Xa && p4.X >= Xa && Math.Min(p1.Y, p2.Y) <= Ya && Math.Max(p1.Y, p2.Y) >= Ya)
                {
                    return new Point(Math.Round(Xa, 4), Math.Round(Ya, 4));
                }
                return null;
            }
            // If the second segment is vertical
            if (p3.X - p4.X == 0)
            {
                // Find Xa, Ya - intersection points of two lines
                Xa = p3.X;
                A1 = (p1.Y - p2.Y) / (p1.X - p2.X);
                b1 = p1.Y - A1 * p1.X;
                Ya = A1 * Xa + b1;
                if (p1.X <= Xa && p2.X >= Xa && Math.Min(p3.Y, p4.Y) <= Ya && Math.Max(p3.Y, p4.Y) >= Ya)
                {
                    return new Point(Math.Round(Xa, 4), Math.Round(Ya, 4));
                }
                return null;
            }
            // The both segments isn't vertical
            A1 = (p1.Y - p2.Y) / (p1.X - p2.X);
            A2 = (p3.Y - p4.Y) / (p3.X - p4.X);
            b1 = p1.Y - A1 * p1.X;
            b2 = p3.Y - A2 * p3.X;
            if (A1 == A2)
            {
                // Parallel segments
                return null;
            }
            // Xa - abscissa of the point of intersection of two lines
            Xa = (b2 - b1) / (A1 - A2);
            Ya = A1 * Xa + b1;
            if ((Xa < Math.Max(p1.X, p3.X)) || (Xa > Math.Min(p2.X, p4.X)))
            {
                // Point Xa is outside the intersection of the projections of the line segments on the X axis
                return null;
            }
            else
            {
                return new Point(Math.Round(Xa, 4), Math.Round(Ya, 4));
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="vertexes"></param>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        public List<Point> CollisionLineAndPolygon(List<Point> vertices, Point p1, Point p2)
        {
            var collisionLineAndPolygonPoints = new List<Point>();
            for (int i = 0; i < vertices.Count; i++)
            {
                if (i < vertices.Count - 1)
                {
                    // The claculating continues while "i" isn't last element of the list
                    Point? pointCrossingLines = CrossingLines(vertices[i], vertices[i + 1], p1, p2);
                    if (pointCrossingLines != null)
                    {
                        collisionLineAndPolygonPoints.Add((Point)pointCrossingLines);
                    }
                }
                else
                {
                    // If "i" is the last element
                    Point? pointCrossingLines = CrossingLines(vertices[i], vertices[0], p1, p2);
                    if (pointCrossingLines != null)
                    {
                        collisionLineAndPolygonPoints.Add((Point)pointCrossingLines);
                    }
                }
            }
            // Remove duplicates if the line passes through the vertices (vertex) of the polygon
            return collisionLineAndPolygonPoints.Distinct().ToList();
        }

        public double LengthSection(Point p1, Point p2)
        {
            return (p2 - p1).Length;
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public abstract class Polygon
    {
        private readonly List<Point> _vertexes;
        private readonly List<Vector> _edges;

        public List<Point> Vertexes
        {
            get => _vertexes;
        }
        public List<Vector> Edges
        {
            get => _edges;
        }

        protected Polygon(List<Point> vertexes)
        {
            _vertexes = vertexes;
            _edges = CalculateEdges(vertexes);
        }

        /// <summary>
        /// Calculates the edges of a convex polygon from specified points
        /// </summary>
        /// <param name="vertexes"></param>
        /// <returns></returns>        
        protected List<Vector> CalculateEdges(List<Point> vertexes)
        {
            var listOfEdges = new List<Vector>();
            for (int i = 0; i < vertexes.Count; i++)
            {
                if (i < vertexes.Count - 1)
                {
                    // The claculating continues while "i" isn't last element of the list
                    listOfEdges.Add(vertexes[i + 1] - vertexes[i]);
                }
                else
                {
                    // If "i" is the last element
                    listOfEdges.Add(vertexes[0] - vertexes[i]);
                }
            }
            return listOfEdges;
        }
    }

    /// <summary>
    /// Convex polygon
    /// </summary>
    public class Convex : Polygon
    {
        /// <summary>
        /// Constructor
        /// </summary>
        public Convex(List<Point> vertexes) : base(vertexes)
        {
            if (!IsConvex(Vertexes, Edges))
            {
                throw new Exception("The polygon is not convex.");
            }
        }
        /// <summary>
        /// Determines whether the geometric figure is a convex polygon
        /// </summary>
        /// <param name="vertexes"></param>
        /// <param name="edges"></param>
        /// <returns>Return true if geometric figure is a convex polygon</returns>
        private bool IsConvex(List<Point> vertexes, List<Vector> edges)
        {
            uint standartSumAnglesConvex = 180 * ((uint)vertexes.Count() - 2);
            double calculateSumAnglesConvex = 0;
            for (int i = 0; i < edges.Count; i++)
            {
                if (i < edges.Count - 1)
                {
                    // The claculating continues while "i" isn't last element of the list
                    calculateSumAnglesConvex += Vector.AngleBetween(edges[i], edges[i + 1]);
                }
                else
                {
                    // If "i" is the last element, calculate last and first edges angle
                    calculateSumAnglesConvex += Vector.AngleBetween(edges[i], edges[0]);
                }
            }
            return standartSumAnglesConvex == (uint)Math.Round(Math.Abs(calculateSumAnglesConvex));
        }
    }

    /// <summary>
    /// Linear object which has width and contain linear equation
    /// </summary>
    public class LinearObject
    {
        private readonly uint _widthLine;
        private readonly LinearEquation _linearEquation;

        public uint Width
        {
            get => _widthLine;
        }
        public LinearEquation LinearEquationInstance
        {
            get => _linearEquation;
        }

        public LinearObject(uint width, int coefficientA, int coefficientB, int coefficientC)
        {
            _widthLine = width;
            _linearEquation = new LinearEquation(coefficientA, coefficientB, coefficientC);
        }
    }

    /// <summary>
    /// Contain formula Ax + By + C = 0 and method of calculating XY of the linear equation.
    /// If the expressions _x and _y are not defined, then the calculations are performed 
    /// according to the general formula.
    /// </summary>
    public class LinearEquation
    {
        private readonly double _coefficientA;
        private readonly double _coefficientB;
        private readonly double _coefficientC;
        private readonly double? _x;
        private readonly double? _y;

        public double CoefficientA
        {
            get => _coefficientA;
        }
        public double CoefficientB
        {
            get => _coefficientB;
        }
        public double CoefficientC
        {
            get => _coefficientC;
        }
        public double? X
        {
            get => _x;
        }
        public double? Y
        {
            get => _y;
        }

        /// <summary>
        /// Constructor of linear equation if A = B = 0 will get the exception
        /// </summary>
        /// <param name="coefficientA">A</param>
        /// <param name="coefficientB">B</param>
        /// <param name="coefficientC">C</param>
        public LinearEquation(double coefficientA, double coefficientB, double coefficientC)
        {
            _coefficientA = coefficientA;
            _coefficientB = coefficientB;
            _coefficientC = coefficientC;
            if ((CoefficientA == 0) && (CoefficientB == 0))
            {
                throw new Exception("The expression A = B = 0 doesn't make sense.");
            }
            else
            {
                if (CoefficientA == 0)
                {
                    _y = -(coefficientC / coefficientB);
                }
                else if (CoefficientB == 0)
                {
                    _x = -(coefficientC / coefficientA);
                }
            }
        }

        /// <summary>
        /// A point on the plane is calculated by linear equation taking into account special cases
        /// </summary>
        /// <param name="coordinate">Some coordinate "x" or "y"</param>
        /// <returns>
        /// If _x = const, then return point (_x ; coordinate)
        /// If _y = const, then return point (coordinate ; _y)
        /// If y = const1 - const2 * x, then return point (x ; y)
        /// </returns>
        public Point CalculatePoint(double coordinate)
        {
            var calculatedPoint = new Point();
            if ((X == null) && (Y == null))
            {
                calculatedPoint.X = Math.Round(coordinate, 4);
                calculatedPoint.Y = Math.Round((-(CoefficientC / CoefficientB) - (CoefficientA / CoefficientB)
                    * calculatedPoint.X), 4); 
            }
            else
            {
                if (X != null)
                {
                    calculatedPoint.X = (double)X;
                    calculatedPoint.Y = coordinate;
                }
                else if (Y != null)
                {
                    calculatedPoint.X = coordinate;
                    calculatedPoint.Y = (double)Y;
                }
            }
            return calculatedPoint;
        }
    }
}
