use embedded_graphics::{
    geometry::AnchorPoint,
    mono_font::{MonoTextStyle, ascii::FONT_8X13},
    pixelcolor::Rgb565,
    prelude::*,
    primitives::{PrimitiveStyleBuilder, Rectangle, StrokeAlignment},
    text::{Alignment, Text},
};

/// Draw text on the display with background clearing
///
/// # Parameters
/// * `text` - The text to draw
/// * `bounding_box` - The rectangle area where to draw
/// * `bg_color` - Background color to fill
/// * `text_color` - Color of the text
/// * `display` - Mutable reference to the display
pub fn draw_text<D>(
    text: &str,
    bounding_box: Rectangle,
    bg_color: Rgb565,
    text_color: Rgb565,
    display: &mut D,
) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // Clear the background first
    display.fill_solid(&bounding_box, bg_color)?;

    // Draw the text
    let character_style = MonoTextStyle::new(&FONT_8X13, text_color);
    Text::with_alignment(
        text,
        bounding_box.center(),
        character_style,
        Alignment::Center,
    )
    .draw(display)?;

    Ok(())
}

/// Initialize the display with title and border
///
/// # Parameters
/// * `display` - Mutable reference to the display
/// * `title` - Title text to display at the top
pub fn draw_init_screen<D>(display: &mut D, title: &str) -> Result<(), D::Error>
where
    D: DrawTarget<Color = Rgb565>,
{
    // Clear the display
    display.clear(Rgb565::BLACK)?;

    // Draw border
    let border_stroke = PrimitiveStyleBuilder::new()
        .stroke_color(Rgb565::BLUE)
        .stroke_width(1)
        .stroke_alignment(StrokeAlignment::Inside)
        .build();
    display
        .bounding_box()
        .into_styled(border_stroke)
        .draw(display)?;

    // Draw title
    let character_style = MonoTextStyle::new(&FONT_8X13, Rgb565::WHITE);
    Text::with_alignment(
        title,
        display.bounding_box().anchor_point(AnchorPoint::TopCenter) + Point::new(0, 15),
        character_style,
        Alignment::Center,
    )
    .draw(display)?;

    Ok(())
}
