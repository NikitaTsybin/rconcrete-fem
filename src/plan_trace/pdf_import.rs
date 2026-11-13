//! Platform-independent PDF bytes -> RGBA pages. No filesystem or window APIs.
pub struct Page {
    pub width: u32,
    pub height: u32,
    pub rgba: Vec<u8>,
    pub size_mm: [f64; 2],
}
fn parse(bytes: Vec<u8>) -> Result<hayro::hayro_syntax::Pdf, String> {
    if bytes.len() > 64 * 1024 * 1024 {
        return Err("PDF превышает 64 МБ".into());
    }
    let pdf = hayro::hayro_syntax::Pdf::new(bytes).map_err(|e| format!("PDF: {e:?}"))?;
    if pdf.pages().is_empty() || pdf.pages().len() > 100 {
        return Err("PDF: требуется от 1 до 100 страниц".into());
    }
    Ok(pdf)
}
/// Metadata only: do not rasterize unused pages before the user chooses one.
pub fn page_count(bytes: &[u8]) -> Result<usize, String> {
    Ok(parse(bytes.to_vec())?.pages().len())
}
pub fn decode_page(bytes: Vec<u8>, index: usize) -> Result<Page, String> {
    decode_selected(bytes, Some(index))?
        .pop()
        .ok_or_else(|| "Страница PDF не найдена".into())
}
pub fn decode(bytes: Vec<u8>) -> Result<Vec<Page>, String> {
    decode_selected(bytes, None)
}
fn decode_selected(bytes: Vec<u8>, selected: Option<usize>) -> Result<Vec<Page>, String> {
    use hayro::{hayro_interpret::InterpreterSettings, RenderCache, RenderSettings};
    let pdf = parse(bytes)?;
    if selected.is_some_and(|i| i >= pdf.pages().len()) {
        return Err("Страница PDF не найдена".into());
    }
    let cache = RenderCache::new();
    let interpreter = InterpreterSettings::default();
    let mut pages = vec![];
    let mut total = 0usize;
    for (index, page) in pdf.pages().iter().enumerate() {
        if selected.is_some_and(|i| i != index) {
            continue;
        }
        let (width, height) = page.render_dimensions();
        if !width.is_finite() || !height.is_finite() || width <= 0. || height <= 0. {
            return Err("PDF: некорректный размер страницы".into());
        }
        let factor = 2048. / width.max(height);
        let settings = RenderSettings {
            x_scale: factor,
            y_scale: factor,
            width: Some((width * factor).round().max(1.) as u16),
            height: Some((height * factor).round().max(1.) as u16),
            bg_color: hayro::vello_cpu::color::palette::css::WHITE,
        };
        let pixmap = hayro::render(page, &cache, &interpreter, &settings);
        let rgba = pixmap.data_as_u8_slice().to_vec();
        total += rgba.len();
        if total > 64 * 1024 * 1024 {
            return Err("PDF: растровые страницы превышают 64 МБ".into());
        }
        pages.push(Page {
            width: pixmap.width() as u32,
            height: pixmap.height() as u32,
            rgba,
            size_mm: [width as f64 * 25.4 / 72., height as f64 * 25.4 / 72.],
        });
    }
    Ok(pages)
}

#[cfg(test)]
pub(super) fn test_pdf(count: usize) -> Vec<u8> {
    let kids = (0..count)
        .map(|i| format!("{} 0 R", 3 + 2 * i))
        .collect::<Vec<_>>()
        .join(" ");
    let mut objects = vec![
        "<< /Type /Catalog /Pages 2 0 R >>".to_owned(),
        format!("<< /Type /Pages /Kids [{kids}] /Count {count} >>"),
    ];
    for i in 0..count {
        let (w, h, color) = if i % 2 == 0 {
            (200, 100, "1 0 0")
        } else {
            (100, 200, "0 1 0")
        };
        let content = format!("{color} rg 0 0 {w} {h} re f");
        objects.push(format!(
            "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {w} {h}] /Contents {} 0 R >>",
            4 + 2 * i
        ));
        objects.push(format!(
            "<< /Length {} >>\nstream\n{content}\nendstream",
            content.len()
        ));
    }
    let mut bytes = b"%PDF-1.4\n".to_vec();
    let mut offsets = vec![0];
    for (i, object) in objects.iter().enumerate() {
        offsets.push(bytes.len());
        bytes.extend(format!("{} 0 obj\n{object}\nendobj\n", i + 1).bytes());
    }
    let start = bytes.len();
    bytes.extend(format!("xref\n0 {}\n0000000000 65535 f \n", offsets.len()).bytes());
    for n in &offsets[1..] {
        bytes.extend(format!("{n:010} 00000 n \n").bytes());
    }
    bytes.extend(
        format!(
            "trailer\n<< /Size {} /Root 1 0 R >>\nstartxref\n{start}\n%%EOF",
            offsets.len()
        )
        .bytes(),
    );
    bytes
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn page_decoder_renders_first_page_and_rejects_invalid_inputs() {
        let bytes = test_pdf(20);
        assert_eq!(page_count(&bytes).unwrap(), 20);
        let first = decode_page(bytes.clone(), 0).unwrap();
        assert_eq!((first.width, first.height), (2048, 1024));
        assert_eq!(&first.rgba[..4], &[255, 0, 0, 255]);
        assert!(decode_page(bytes, 20).is_err());
        assert!(page_count(b"not a PDF").is_err());
    }
    #[test]
    fn renders_pdf_vectors_and_standard_text_without_platform_libraries() {
        let content = "1 0 0 rg 0 0 200 100 re f 0 g BT /F1 12 Tf 20 50 Td (PDF test) Tj ET";
        let objects=[
            "<< /Type /Catalog /Pages 2 0 R >>".to_string(),
            "<< /Type /Pages /Kids [3 0 R] /Count 1 >>".to_string(),
            "<< /Type /Page /Parent 2 0 R /MediaBox [0 0 200 100] /Resources << /Font << /F1 5 0 R >> >> /Contents 4 0 R >>".to_string(),
            format!("<< /Length {} >>\nstream\n{content}\nendstream",content.len()),
            "<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>".to_string(),
        ];
        let mut bytes = b"%PDF-1.4\n".to_vec();
        let mut offsets = vec![0];
        for (i, o) in objects.iter().enumerate() {
            offsets.push(bytes.len());
            bytes.extend(format!("{} 0 obj\n{o}\nendobj\n", i + 1).bytes());
        }
        let start = bytes.len();
        bytes.extend(format!("xref\n0 {}\n0000000000 65535 f \n", offsets.len()).bytes());
        for n in &offsets[1..] {
            bytes.extend(format!("{n:010} 00000 n \n").bytes());
        }
        bytes.extend(
            format!(
                "trailer\n<< /Size {} /Root 1 0 R >>\nstartxref\n{start}\n%%EOF",
                offsets.len()
            )
            .bytes(),
        );
        let pages = decode(bytes).unwrap();
        assert_eq!(pages.len(), 1);
        let p = &pages[0];
        assert_eq!((p.width, p.height), (2048, 1024));
        assert!((p.size_mm[0] - 200. * 25.4 / 72.).abs() < 1e-5);
        assert_eq!(&p.rgba[0..4], &[255, 0, 0, 255]);
        assert!(p
            .rgba
            .chunks_exact(4)
            .any(|c| c[0] < 20 && c[1] < 20 && c[2] < 20));
        assert!(decode(b"not a PDF".to_vec()).is_err());
    }
}
