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
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public class Program
    {
        public Program()
        {
            var convex = new Convex(new List<Point>());
            var line1 = new LinearObject(1, 5, 5, 7);
            var line2 = new LinearObject(2, 2, 1, 1);
            var linearsObjects = new List<LinearObject>();
            linearsObjects.Add(line1);
            linearsObjects.Add(line2);
            //Написать метод в текущем классе, который будет определять точки коллизии между 
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public abstract class Polygon
    {
        private List<Point> _vertexes;
        private List<Vector> _edges;

        public List<Point> Vertexes
        {
            get => _vertexes;
        }
        public List<Vector> Edges
        {
            get => _edges;
        }

        /// <summary>
        /// The initializer method is used instead of initializing the class fields in the constructor because 
        /// derived classes may need to compute 
        /// </summary>
        /// <param name="vertexes"></param>
        protected void Initialize(List<Point> vertexes)
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
                    // If "i" is the last element, calculate last and first edges angle
                    listOfEdges.Add(vertexes[0] - vertexes[i - 1]);
                }
            }
            return listOfEdges;
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public class Convex : Polygon
    {
        /// <summary>
        /// Constructor
        /// </summary>
        public Convex(List<Point> vertexes)
        {
            if (!IsConvex(Vertexes, Edges))
            {
                throw new Exception("The polygon is not convex.");
            }
            Initialize(vertexes);
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
            return standartSumAnglesConvex == (uint)Math.Round(calculateSumAnglesConvex);
        }
    }

    /// <summary>
    /// 
    /// </summary>
    public class LinearObject : Polygon
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
            //Initialize(CreateVertexes());
        }

        //public List<Point> CreateVertexes()
        //{
        //    var a = LinearEquationInstance;
        //    return new List<Point>();
        //}
    }

    /// <summary>
    /// Contain formula Ax + By + C = 0 and method of calculating XY of the linear equation.
    /// If the expressions _x and _y are not defined, then the calculations are performed 
    /// according to the general formula.
    /// </summary>
    public class LinearEquation
    {
        private readonly int _coefficientA;
        private readonly int _coefficientB;
        private readonly int _coefficientC;
        private readonly int? _x;
        private readonly int? _y;

        public int CoefficientA
        {
            get => _coefficientA;
        }
        public int CoefficientB
        {
            get => _coefficientB;
        }
        public int CoefficientC
        {
            get => _coefficientC;
        }
        public int? X
        {
            get => _x;
        }
        public int? Y
        {
            get => _y;
        }

        /// <summary>
        /// Constructor of linear equation if A = B = 0 will get the exception
        /// </summary>
        /// <param name="coefficientA">A</param>
        /// <param name="coefficientB">B</param>
        /// <param name="coefficientC">C</param>
        public LinearEquation(int coefficientA, int coefficientB, int coefficientC)
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
        public Point CalculatePoint(int coordinate)
        {
            var calculatedPoint = new Point();
            if ((X == null) && (Y == null))
            {
                calculatedPoint.X = coordinate;
                calculatedPoint.Y = (CoefficientC / CoefficientB) - (CoefficientA / CoefficientB) * calculatedPoint.X;
            }
            else
            {
                if (X != null)
                {
                    calculatedPoint.X = (int)X;
                    calculatedPoint.Y = coordinate;
                }
                else if (Y != null)
                {
                    calculatedPoint.X = coordinate;
                    calculatedPoint.Y = (int)Y;
                }
            }
            return calculatedPoint;
        }
    }
}
