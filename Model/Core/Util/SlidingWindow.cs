using System.Collections;

namespace Model.Core.Util;


/// <summary>
/// Represents a fixed-size circular buffer (sliding window).
/// When new elements are added beyond capacity, the oldest elements are overwritten.
/// </summary>
/// <typeparam name="T">Type of elements stored in the window.</typeparam>
public sealed class SlidingWindow<T> : IEnumerable<T>
{
    private readonly T[] _items;
    private int _next = 0;

    /// <summary>
    /// Gets the maximum number of elements the window can hold.
    /// </summary>
    public int Capacity => _items.Length;
    public int Count { get; private set; } = 0;

    /// <summary>
    /// Initializes a new instance of the <see cref="SlidingWindow{T}"/> class.
    /// </summary>
    /// <param name="size">The fixed size of the sliding window. Must be greater than zero.</param>
    /// <exception cref="ArgumentOutOfRangeException">Thrown when size is less than or equal to zero.</exception>
    public SlidingWindow(int size)
    {
        if (size <= 0)
            throw new ArgumentOutOfRangeException(nameof(size), "Size must be greater than zero");

        _items = new T[size];
    }

    /// <summary>
    /// Adds a new element to the window.
    /// If the window is full, the oldest element is overwritten.
    /// </summary>
    /// <param name="value">The element to add.</param>
    /// <returns>
    /// Oldest element in the window if <see cref="Count"/>=<see cref="Capacity"> otherwise no-value
    /// </returns>
    public T? Push(T value)
    {
        var oldest = _items[_next];
        _items[_next] = value;
        _next = (_next + 1) % Capacity;

        if (Count < Capacity)
        {
            ++Count;
            return default;
        }
        return oldest;
    }

    /// <summary>
    /// Gets the element at the specified index in the window.
    /// Index 0 corresponds to the oldest element,
    /// and <see cref="Count"/> - 1 corresponds to the most recently added element.
    /// </summary>
    /// <param name="i">Index in the logical window (0 = oldest).</param>
    /// <returns>The element at the specified index.</returns>
    /// <exception cref="ArgumentOutOfRangeException">Thrown when index is out of range.</exception>
    public T this[int i] {
        get {
            if (i < 0 || i >= Count)
                throw new ArgumentOutOfRangeException(nameof(i));
            int start = Count == Capacity ? _next : 0;
            return _items[(start + i) % Capacity];
        }
    }

    /// <summary>
    /// Returns an enumerator that iterates through the window
    /// from oldest to newest element.
    /// </summary>
    public IEnumerator<T> GetEnumerator()
    {
        for (int i = 0; i < Count; i++)
            yield return this[i];
    }

    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();

    public void Clear()
    {
        Array.Clear(_items);
        _next = 0;
        Count = 0;
    }
}
