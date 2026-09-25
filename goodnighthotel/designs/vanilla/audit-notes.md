# Migration notes

## Source mapping

The fourteen pages map as follows. French routes remain at the root; English routes are normalized under `/en/`.

| Generated page | Mirrored source |
|---|---|
| `/index.html` | `www.good-night-hotel.com/index.html` |
| `/chambres-et-petit-dejeuner/index.html` | `www.good-night-hotel.com/chambres-et-petit-dejeuner/index.html` |
| `/informations-et-services/index.html` | `www.good-night-hotel.com/informations-et-services/index.html` |
| `/lieux-touristiques/index.html` | `www.good-night-hotel.com/lieux-touristiques/index.html` |
| `/promotions-et-offres-speciales/index.html` | `www.good-night-hotel.com/promotions-et-offres-speciales/index.html` |
| `/contact/index.html` | `www.good-night-hotel.com/contact/index.html` |
| `/mentions-legales/index.html` | `www.good-night-hotel.com/mentions-legales/index.html` |
| `/en/index.html` | `www.good-night-hotel.co.uk/index.html` |
| `/en/room-and-breakfast/index.html` | `www.good-night-hotel.co.uk/room-and-breakfast/index.html` |
| `/en/informations-and-services/index.html` | `www.good-night-hotel.co.uk/informations-and-services/index.html` |
| `/en/tourist-places/index.html` | `www.good-night-hotel.co.uk/tourist-places/index.html` |
| `/en/special-offers/index.html` | `www.good-night-hotel.co.uk/special-offers/index.html` |
| `/en/contact-us/index.html` | `www.good-night-hotel.co.uk/contact-us/index.html` |
| `/en/legal-mention/index.html` | `www.good-night-hotel.co.uk/legal-mention/index.html` |

## Migrated assets

Branding, hero, hotel/room imagery, tourism imagery, reservation backgrounds, map/sidebar image, footer/header textures, icons, and promotion buttons were copied into `assets/images/`. Original source files were not modified.

## Removed dependencies

The rebuild does not reference WordPress paths, WPML, Contact Form 7, jQuery, EasySlider, `l10n.js`, XML-RPC, feeds, or analytics. The old fixed-width theme CSS was used only as visual reference and replaced by `assets/css/main.css`.

## Booking

The public mirror exposed both a legacy Reservit GET form (`hotelid=172781`) and a BookingButton iframe for `goodnightdirect` with French/English locales. Both are preserved for baseline verification; Reservit is labelled legacy and includes a production TODO.

## Known source issues

The mirror contained malformed duplicated protocol/path fragments, mojibake text, stale date options, relative `index.html` canonicals, and a missing `html5-ie.js` reference. The new project uses UTF-8, clean local paths, semantic markup, and placeholder absolute canonical URLs with language alternates.

## Unresolved

The current deployment/canonical domain strategy, contact delivery endpoint, analytics consent requirements, Reservit status, and current SiteMinder contract still require hotel confirmation.
