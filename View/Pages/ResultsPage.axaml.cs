using System.Globalization;
using System.Runtime.InteropServices;
using System.Reactive;
using System.Reactive.Linq;
using App.ViewModels;
using Avalonia;
using Avalonia.Controls.Primitives;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Media;
using Avalonia.Media.Imaging;
using Avalonia.Platform;
using Avalonia.Threading;
using ReactiveUI;
using ReactiveUI.Avalonia;

namespace App.Pages;

public partial class ResultsPage : ReactiveUserControl<MainWindowViewModel>
{
    private const double SpatialWidth = 780;
    private const double SpatialHeight = 520;
    private const double DrawMargin = 24;
    private const double VertexPickRadius = 12.0;

    private bool _isUpdatingControls;
    private int _selectedVertexIndex;
    private Point[] _lastMappedPoints = [];
    private bool _hasSnapshot;

    public ResultsPage()
    {
        InitializeComponent();
        this.WhenActivated(disposables =>
        {
            var viewModelUpdates = this.WhenAnyValue(v => v.ViewModel)
                .Select(vm => vm is null
                    ? Observable.Empty<Unit>()
                    : vm.WhenAnyValue(x => x.ResultRevision)
                        .StartWith(vm.ResultRevision)
                        .Select(_ => Unit.Default))
                .Switch();

            var layerUpdates = LayerSlider
                .GetObservable(RangeBase.ValueProperty)
                .Where(_ => !_isUpdatingControls)
                .Select(_ => Unit.Default);

            var refreshSubscription = Observable
                .Merge(viewModelUpdates, layerUpdates)
                .Subscribe(_ => Dispatcher.UIThread.Post(() => RefreshVisualization(resetSelection: false)));
            disposables.Add(refreshSubscription);
        });
    }

    private void OnPrevLayerClick(object? sender, RoutedEventArgs e)
    {
        if (LayerSlider.Value <= LayerSlider.Minimum)
            return;

        LayerSlider.Value = Math.Max(LayerSlider.Minimum, LayerSlider.Value - 1);
    }

    private void OnNextLayerClick(object? sender, RoutedEventArgs e)
    {
        if (LayerSlider.Value >= LayerSlider.Maximum)
            return;

        LayerSlider.Value = Math.Min(LayerSlider.Maximum, LayerSlider.Value + 1);
    }

    private void OnSpatialImagePointerPressed(object? sender, PointerPressedEventArgs e)
    {
        if (!_hasSnapshot || _lastMappedPoints.Length == 0)
            return;

        if (!e.GetCurrentPoint(SpatialImage).Properties.IsLeftButtonPressed)
            return;

        var click = e.GetPosition(SpatialImage);
        var bestIndex = -1;
        var bestDist2 = double.MaxValue;
        for (int i = 0; i < _lastMappedPoints.Length; i++)
        {
            var dx = _lastMappedPoints[i].X - click.X;
            var dy = _lastMappedPoints[i].Y - click.Y;
            var dist2 = dx * dx + dy * dy;
            if (dist2 < bestDist2)
            {
                bestDist2 = dist2;
                bestIndex = i;
            }
        }

        if (bestIndex < 0 || bestDist2 > VertexPickRadius * VertexPickRadius)
            return;

        _selectedVertexIndex = bestIndex;
        RefreshVisualization(resetSelection: false);
        e.Handled = true;
    }

    private void RefreshVisualization(bool resetSelection)
    {
        var vm = ViewModel;
        if (vm is null)
            return;

        var layerIndex = resetSelection ? 0 : (int)Math.Round(LayerSlider.Value);
        var vertexIndex = resetSelection ? 0 : _selectedVertexIndex;

        if (!vm.TryGetResultVisualizationSnapshot(layerIndex, vertexIndex, out var snapshot, out var message))
        {
            _isUpdatingControls = true;
            try
            {
                LayerSlider.Minimum = 0;
                LayerSlider.Maximum = 0;
                LayerSlider.Value = 0;
            }
            finally
            {
                _isUpdatingControls = false;
            }

            LayerStatusText.Text = message;
            SelectedVertexText.Text = "—";
            SpatialImage.Source = null;
            _lastMappedPoints = [];
            _hasSnapshot = false;
            return;
        }

        _isUpdatingControls = true;
        try
        {
            var maxLayer = Math.Max(0, snapshot.LayerTimes.Length - 1);
            LayerSlider.Minimum = 0;
            LayerSlider.Maximum = maxLayer;
            LayerSlider.Value = snapshot.LayerIndex;
        }
        finally
        {
            _isUpdatingControls = false;
        }

        _selectedVertexIndex = snapshot.SelectedVertexIndex;
        LayerStatusText.Text = message;
        SelectedVertexText.Text = $"#{snapshot.SelectedVertexIndex}: u={snapshot.Values[snapshot.SelectedVertexIndex].ToString("G6", CultureInfo.InvariantCulture)}";

        SpatialImage.Source = BuildSpatialImage(snapshot, out var mappedPoints);
        _lastMappedPoints = mappedPoints;
        _hasSnapshot = true;
    }

    private static IImage BuildSpatialImage(
        MainWindowViewModel.ResultVisualizationSnapshot snapshot,
        out Point[] mappedPoints)
    {
        var drawing = new DrawingGroup();
        mappedPoints = BuildMappedPoints(snapshot.X, snapshot.Y, SpatialWidth, SpatialHeight, out _);
        var raster = BuildSmoothFieldBitmap(snapshot, mappedPoints);
        drawing.Children.Add(new ImageDrawing
        {
            ImageSource = raster,
            Rect = new Rect(0, 0, SpatialWidth, SpatialHeight)
        });

        var edgePen = new Pen(new SolidColorBrush(Color.FromRgb(35, 35, 35)), 1.2);
        var edgeSet = new HashSet<long>();

        foreach (var element in snapshot.Elements)
        {
            if (element.Length < 2)
                continue;

            for (int i = 0; i < element.Length; i++)
            {
                var a = element[i];
                var b = element[(i + 1) % element.Length];
                if (!IsValidVertexIndex(a, snapshot.X.Length) || !IsValidVertexIndex(b, snapshot.X.Length))
                    continue;

                var key = MakeEdgeKey(a, b);
                if (!edgeSet.Add(key))
                    continue;

                drawing.Children.Add(new GeometryDrawing
                {
                    Pen = edgePen,
                    Geometry = new LineGeometry(mappedPoints[a], mappedPoints[b])
                });
            }
        }

        for (int i = 0; i < snapshot.Boundaries.Length; i++)
        {
            var boundary = snapshot.Boundaries[i];
            if (boundary.Length < 2)
                continue;

            var a = boundary[0];
            var b = boundary[1];
            if (!IsValidVertexIndex(a, snapshot.X.Length) || !IsValidVertexIndex(b, snapshot.X.Length))
                continue;

            var boundaryIndex = i < snapshot.BoundaryIndices.Length ? snapshot.BoundaryIndices[i] : 0;
            var boundaryPen = new Pen(new SolidColorBrush(BoundaryColor(boundaryIndex)), 2.8);
            drawing.Children.Add(new GeometryDrawing
            {
                Pen = boundaryPen,
                Geometry = new LineGeometry(mappedPoints[a], mappedPoints[b])
            });
        }

        var selectedPoint = mappedPoints[snapshot.SelectedVertexIndex];
        drawing.Children.Add(new GeometryDrawing
        {
            Pen = new Pen(Brushes.Black, 2),
            Geometry = new EllipseGeometry(new Rect(selectedPoint.X - 6, selectedPoint.Y - 6, 12, 12))
        });

        DrawLegend(drawing, snapshot);

        return new DrawingImage(drawing);
    }

    private static WriteableBitmap BuildSmoothFieldBitmap(
        MainWindowViewModel.ResultVisualizationSnapshot snapshot,
        Point[] mappedPoints)
    {
        var width = (int)SpatialWidth;
        var height = (int)SpatialHeight;
        var stride = width * 4;
        var pixels = new byte[stride * height];

        FillBackground(pixels, width, height, stride, Color.FromRgb(245, 245, 245));

        foreach (var element in snapshot.Elements)
        {
            if (element.Length < 3)
                continue;

            var vertices = new List<(Point P, double U)>(element.Length);
            foreach (var vertexIndex in element)
            {
                if (!IsValidVertexIndex(vertexIndex, snapshot.X.Length))
                    continue;
                vertices.Add((mappedPoints[vertexIndex], snapshot.Values[vertexIndex]));
            }

            if (vertices.Count < 3)
                continue;

            if (vertices.Count == 3)
            {
                RasterizeTriangle(
                    pixels,
                    width,
                    height,
                    stride,
                    vertices[0].P, vertices[0].U,
                    vertices[1].P, vertices[1].U,
                    vertices[2].P, vertices[2].U,
                    snapshot.MinValue,
                    snapshot.MaxValue);
                continue;
            }

            var centerX = 0.0;
            var centerY = 0.0;
            var centerU = 0.0;
            foreach (var (p, u) in vertices)
            {
                centerX += p.X;
                centerY += p.Y;
                centerU += u;
            }

            var invN = 1.0 / vertices.Count;
            var centerPoint = new Point(centerX * invN, centerY * invN);
            centerU *= invN;

            for (int i = 0; i < vertices.Count; i++)
            {
                var a = vertices[i];
                var b = vertices[(i + 1) % vertices.Count];
                RasterizeTriangle(
                    pixels,
                    width,
                    height,
                    stride,
                    a.P, a.U,
                    b.P, b.U,
                    centerPoint, centerU,
                    snapshot.MinValue,
                    snapshot.MaxValue);
            }
        }

        var bitmap = new WriteableBitmap(
            new PixelSize(width, height),
            new Vector(96, 96),
            PixelFormat.Bgra8888,
            AlphaFormat.Opaque);

        using var framebuffer = bitmap.Lock();
        for (int y = 0; y < height; y++)
        {
            Marshal.Copy(
                pixels,
                y * stride,
                IntPtr.Add(framebuffer.Address, y * framebuffer.RowBytes),
                stride);
        }

        return bitmap;
    }

    private static void FillBackground(byte[] pixels, int width, int height, int stride, Color color)
    {
        for (int y = 0; y < height; y++)
        {
            var row = y * stride;
            for (int x = 0; x < width; x++)
            {
                var idx = row + x * 4;
                pixels[idx + 0] = color.B;
                pixels[idx + 1] = color.G;
                pixels[idx + 2] = color.R;
                pixels[idx + 3] = 255;
            }
        }
    }

    private static void RasterizeTriangle(
        byte[] pixels,
        int width,
        int height,
        int stride,
        Point p0,
        double u0,
        Point p1,
        double u1,
        Point p2,
        double u2,
        double minU,
        double maxU)
    {
        var minX = Math.Min(p0.X, Math.Min(p1.X, p2.X));
        var maxX = Math.Max(p0.X, Math.Max(p1.X, p2.X));
        var minY = Math.Min(p0.Y, Math.Min(p1.Y, p2.Y));
        var maxY = Math.Max(p0.Y, Math.Max(p1.Y, p2.Y));

        var x0 = Math.Clamp((int)Math.Floor(minX), 0, width - 1);
        var x1 = Math.Clamp((int)Math.Ceiling(maxX), 0, width - 1);
        var y0 = Math.Clamp((int)Math.Floor(minY), 0, height - 1);
        var y1 = Math.Clamp((int)Math.Ceiling(maxY), 0, height - 1);
        if (x1 < x0 || y1 < y0)
            return;

        var area = Edge(p0, p1, p2);
        if (Math.Abs(area) < 1e-10)
            return;

        var invArea = 1.0 / area;
        const double eps = 1e-9;

        for (int y = y0; y <= y1; y++)
        {
            var py = y + 0.5;
            var row = y * stride;
            for (int x = x0; x <= x1; x++)
            {
                var px = x + 0.5;
                var p = new Point(px, py);

                var w0 = Edge(p1, p2, p) * invArea;
                var w1 = Edge(p2, p0, p) * invArea;
                var w2 = Edge(p0, p1, p) * invArea;
                if (w0 < -eps || w1 < -eps || w2 < -eps)
                    continue;

                var u = w0 * u0 + w1 * u1 + w2 * u2;
                var color = MapValueToColor(u, minU, maxU);

                var idx = row + x * 4;
                pixels[idx + 0] = color.B;
                pixels[idx + 1] = color.G;
                pixels[idx + 2] = color.R;
                pixels[idx + 3] = 255;
            }
        }
    }

    private static double Edge(Point a, Point b, Point p)
        => (p.X - a.X) * (b.Y - a.Y) - (p.Y - a.Y) * (b.X - a.X);

    private static Point[] BuildMappedPoints(
        double[] x,
        double[] y,
        double width,
        double height,
        out Func<double, double, Point> map)
    {
        var minX = x.Min();
        var maxX = x.Max();
        var minY = y.Min();
        var maxY = y.Max();

        var spanX = Math.Max(1e-12, maxX - minX);
        var spanY = Math.Max(1e-12, maxY - minY);

        var workWidth = width - 2 * DrawMargin;
        var workHeight = height - 2 * DrawMargin;
        var scale = Math.Min(workWidth / spanX, workHeight / spanY);
        var offsetX = (width - spanX * scale) * 0.5 - minX * scale;
        var offsetY = (height - spanY * scale) * 0.5 + maxY * scale;

        map = (px, py) => new Point(offsetX + px * scale, offsetY - py * scale);

        var points = new Point[x.Length];
        for (int i = 0; i < points.Length; i++)
            points[i] = map(x[i], y[i]);
        return points;
    }

    private static void DrawLegend(
        DrawingGroup drawing,
        MainWindowViewModel.ResultVisualizationSnapshot snapshot)
    {
        var width = SpatialWidth;
        var x0 = width - 56;
        var y0 = 30.0;
        var h = 180.0;
        var w = 14.0;
        const int steps = 28;

        for (int i = 0; i < steps; i++)
        {
            var t0 = (double)i / steps;
            var t1 = (double)(i + 1) / steps;
            var value = snapshot.MaxValue - t0 * (snapshot.MaxValue - snapshot.MinValue);
            drawing.Children.Add(new GeometryDrawing
            {
                Brush = new SolidColorBrush(MapValueToColor(value, snapshot.MinValue, snapshot.MaxValue)),
                Geometry = new RectangleGeometry(new Rect(x0, y0 + h * t0, w, h * (t1 - t0)))
            });
        }

        drawing.Children.Add(new GeometryDrawing
        {
            Pen = new Pen(new SolidColorBrush(Color.FromRgb(40, 40, 40)), 1),
            Geometry = new RectangleGeometry(new Rect(x0, y0, w, h))
        });
    }

    private static bool IsValidVertexIndex(int vertexIndex, int vertexCount)
        => vertexIndex >= 0 && vertexIndex < vertexCount;

    private static long MakeEdgeKey(int a, int b)
    {
        var min = Math.Min(a, b);
        var max = Math.Max(a, b);
        return ((long)min << 32) ^ (uint)max;
    }

    private static Color MapValueToColor(double value, double minValue, double maxValue)
    {
        if (maxValue <= minValue + 1e-12)
            return Color.FromRgb(120, 120, 120);

        var t = (value - minValue) / (maxValue - minValue);
        t = Math.Clamp(t, 0.0, 1.0);

        return t switch
        {
            < 0.25 => Lerp(Color.FromRgb(40, 70, 180), Color.FromRgb(45, 175, 230), t / 0.25),
            < 0.5 => Lerp(Color.FromRgb(45, 175, 230), Color.FromRgb(75, 190, 105), (t - 0.25) / 0.25),
            < 0.75 => Lerp(Color.FromRgb(75, 190, 105), Color.FromRgb(240, 215, 70), (t - 0.5) / 0.25),
            _ => Lerp(Color.FromRgb(240, 215, 70), Color.FromRgb(205, 70, 50), (t - 0.75) / 0.25)
        };
    }

    private static Color BoundaryColor(int boundaryIndex)
    {
        return (boundaryIndex % 6) switch
        {
            0 => Color.FromRgb(192, 57, 43),
            1 => Color.FromRgb(22, 160, 133),
            2 => Color.FromRgb(41, 128, 185),
            3 => Color.FromRgb(142, 68, 173),
            4 => Color.FromRgb(243, 156, 18),
            _ => Color.FromRgb(127, 140, 141)
        };
    }

    private static Color Lerp(Color a, Color b, double t)
    {
        var r = (byte)Math.Clamp(a.R + (b.R - a.R) * t, 0, 255);
        var g = (byte)Math.Clamp(a.G + (b.G - a.G) * t, 0, 255);
        var bl = (byte)Math.Clamp(a.B + (b.B - a.B) * t, 0, 255);
        return Color.FromRgb(r, g, bl);
    }
}
