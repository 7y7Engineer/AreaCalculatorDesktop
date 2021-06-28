using System.Collections.Generic;
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

            //Polygon convex = new Convex(new List<Point>() { new Point(0, -10), new Point(0, 10), new Point(20, 10), new Point(20, -10) });
            //var line1 = new LinearObject(1, 5, 5, -7); // 5x\ +\ 5y\ -\ 7\ =0
            //var line2 = new LinearObject(2, 2, 5, -6); // 2x\ +\ 5y\ -\ 6\ =0
            //var line3 = new LinearObject(1, -7, 1, 30); // -7x\ +\ y\ + \ 30\ =0
            //var linearObjects = new List<LinearObject> { line1, line2, line3 };

            Polygon convex = new Convex(new List<Point>() { new Point(0, 0), new Point(0, 10), new Point(10, 10), new Point(10, 0) });
            var line1 = new LinearObject(1, 0, 1, -5);
            var line2 = new LinearObject(1, 1, 0, -5);
            var linearObjects = new List<LinearObject> { line1, line2 };

            var areaCalculator = new AreaCalculator(convex, linearObjects);
            // For a more accurate meaning, change second parametr rounding of Round()
            var resultSquare = areaCalculator.SquareLinesInPolygon();
        }
    }
}