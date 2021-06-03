using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;

namespace AreaCalculatorDesktop
{
    /// <summary>
    /// Calculates the area of ​​crossing lines 
    /// </summary>
    public class AreaCalculator
    {
        private readonly Polygon _polygon;
        private readonly List<LinearObject> _linearObjects;

        public Polygon Polygon
        {
            get => _polygon;
        }
        public List<LinearObject> LinearObjects
        {
            get => _linearObjects;
        }

        // Cartesian plane constants 
        public const double c_min_coordinate = -100;
        public const double c_max_coordinate = 100;

        public AreaCalculator(Polygon polygon, List<LinearObject> linearObjects)
        {
            _polygon = polygon;
            _linearObjects = linearObjects;
        }

        /// <summary>
        /// Checks if two line segments crossing [p1, p2] and [p3, p4]
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <param name="p3"></param>
        /// <param name="p4"></param>
        /// <returns> 
        /// Return null if lines isn't crossing
        /// Return point of crossing, if lines is crossing
        /// </returns>
        protected Point? CrossingLines(Point p1, Point p2, Point p3, Point p4)
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
        /// List of all sections witch cross polygon
        /// </summary>
        /// <param name="verticesPolygon"></param>
        /// <param name="linearObjects"></param>
        /// <returns></returns>
        protected List<List<Point>> CrossPolygonLines(List<Point> verticesPolygon, List<LinearObject> linearObjects)
        {
            var crossPolygonLines = new List<List<Point>>();
            foreach (var line in linearObjects)
            {
                crossPolygonLines.Add(CollisionLineAndPolygon(verticesPolygon, line.LinearEquationInstance.CalculatePoint(c_min_coordinate),
                    line.LinearEquationInstance.CalculatePoint(c_max_coordinate)));
            }
            return crossPolygonLines;
        }

        /// <summary>
        /// List of points section witch cross edge polygon
        /// </summary>
        /// <param name="vertexes"></param>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        protected List<Point> CollisionLineAndPolygon(List<Point> vertices, Point p1, Point p2)
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

        /// <summary>
        /// Calculate length section 
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        protected double LengthSection(Point p1, Point p2)
        {
            return (p2 - p1).Length;
        }

        /// <summary>
        /// Square of crossing polygon lines
        /// </summary>
        /// <param name="linearObjects">linearObjects collection</param>
        /// <param name="crossPolygonLines">crossPolygonLines collection</param>
        /// <returns></returns>
        protected double SquareLinesCrossPolygonWithoutOverlap(List<List<Point>> crossPolygonLines, List<LinearObject> linearObjects)
        {
            double squareLinesCrossPolygonWithoutOverlap = 0;
            for (int i = 0; i < linearObjects.Count; i++)
            {
                squareLinesCrossPolygonWithoutOverlap += Math.Abs(LengthSection(crossPolygonLines[i][0], crossPolygonLines[i][1])
                    * linearObjects[i].Width);
            }
            return Math.Abs(squareLinesCrossPolygonWithoutOverlap);
        }

        /// <summary>
        /// Square of crossing lines overlap
        /// </summary>
        /// <param name="linearObjects">linearObjects collection</param>
        /// <param name="crossPolygonLines">crossPolygonLines collection</param>
        /// <returns></returns>
        protected double SquareOverlapLines(List<List<Point>> crossPolygonLines, List<LinearObject> linearObjects)
        {
            double squareOverlapLines = 0;
            for (int i = 0; i < linearObjects.Count; i++)
            {
                for (int j = i + 1; j < linearObjects.Count; j++)
                {
                    Point? pointCrossingLines = CrossingLines(crossPolygonLines[i][0], crossPolygonLines[i][1],
                        crossPolygonLines[j][0], crossPolygonLines[j][1]);
                    if (pointCrossingLines != null)
                    {
                        // Formula squareOverlapLines += A[i] * B[j] * (1 / sin(AngleAB))
                        squareOverlapLines += linearObjects[i].Width * linearObjects[j].Width * (1 / Math.Abs(Math.Sin(Vector.AngleBetween(
                            crossPolygonLines[i][1] - crossPolygonLines[i][0], crossPolygonLines[j][1] - crossPolygonLines[j][0]))));
                    }
                }
            }
            return Math.Abs(squareOverlapLines);
        }

        /// <summary>
        /// Square of crossing lines in the polygon. Calculating using state object
        /// </summary>
        /// <param name="squareLinesCrossPolygonWithoutOverlap"></param>
        /// <param name="squareOverlapLines"></param>
        /// <returns></returns>
        public double SquareLinesInPolygon()
        {
            var crossPolygonLines = CrossPolygonLines(Polygon.Vertices, LinearObjects);
            return Math.Round(Math.Abs(SquareLinesCrossPolygonWithoutOverlap(crossPolygonLines, LinearObjects)
                - SquareOverlapLines(crossPolygonLines, LinearObjects)));
        }
    }

    /// <summary>
    /// General polygon class
    /// </summary>
    public abstract class Polygon
    {
        private readonly List<Point> _vertexes;
        private readonly List<Vector> _edges;

        public List<Point> Vertices
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
        public Convex(List<Point> vertexes) : base(vertexes)
        {
            if (!IsConvex(Vertices, Edges))
            {
                throw new Exception("The polygon isn't convex.");
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
        private readonly double _widthLine;
        private readonly LinearEquation _linearEquation;

        public double Width
        {
            get => _widthLine;
        }
        public LinearEquation LinearEquationInstance
        {
            get => _linearEquation;
        }

        public LinearObject(double width, int coefficientA, int coefficientB, int coefficientC)
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
