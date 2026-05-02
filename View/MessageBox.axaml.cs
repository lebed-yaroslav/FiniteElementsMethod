using System.Reactive;
using System.Reactive.Linq;
using Avalonia.Controls;
using ReactiveUI;
using ReactiveUI.SourceGenerators;
namespace view;

public partial class TextInfo : ReactiveObject
{
    [Reactive] public partial string Text { get; set; } = string.Empty;
    public ReactiveCommand<Unit, Unit> ReplaceTextCommand {get;}
    
    public TextInfo ()
    {
        ReplaceTextCommand = ReactiveCommand.Create<Unit, Unit>(_ =>
        {
            Text = "ReplacedText";
            return default;
        },
        this.WhenAnyValue(t => t.Text).Select(text=> text == "Hello"));
    }

}

public partial class MessageBox : Window
{
    public MessageBox(string? text = null)
    {
        DataContext = new TextInfo() { Text = text ?? "" };
        InitializeComponent();
    }

    private void Button_Click(object? sender, Avalonia.Interactivity.RoutedEventArgs e)
    {
        Close();
    }
}
