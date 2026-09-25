# Good Night Hotel — design template system

This is a separate single-page design-template system inspired by the supplied VOLNA Residences interaction model and rebuilt with the existing Good Night Hotel palette and local hotel assets.

`template.html` is the pasted VOLNA reference template, preserved for direct visual comparison. `index.html` is the Good Night Hotel implementation of that system using the hotel palette and hotel content/assets.

## Run

```powershell
cd "C:\Users\kkras\OneDrive\Documents\GoodNightHotel\good-night-hotel-vanilla-design-system"
python -m http.server 8081
```

Open <http://localhost:8081/>.

## System

- Tailwind CSS CDN for utility layout classes.
- One custom CSS layer for hotel tokens, motion, noise, glass, cursor glow, and theme overrides.
- Vanilla JavaScript for reveal observers, scroll navigation, theme toggle, card light passes, modal behavior, and canvas hero button effects.
- Hotel palette is documented in `brand-palette.md` in the main vanilla project.
- Uses local Good Night Hotel photography and branding assets; no new imagery was invented.

## Sections

Navigation, hero, philosophy, infrastructure/services, payment ticker, room layouts, lead CTA, footer, and lead modal.

The forms are presentation-only until an approved hotel contact endpoint is connected.
