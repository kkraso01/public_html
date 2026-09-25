# Good Night Hotel Vanilla Rebuild

## Run locally

From this directory:

```powershell
python -m http.server 8080
```

Then open <http://localhost:8080/>.

## Architecture

Static HTML/CSS/vanilla JavaScript. The French site is at the root and the English site is under `/en/`.

## Booking

Current SiteMinder/TheBookingButton property: `goodnightdirect`.

- French locale: `fr`
- English locale: `en`
- Legacy Reservit form is retained in an explicitly labelled disclosure panel.

Analytics intentionally omitted. Confirm GA4/consent requirements with hotel before production.

## Known production TODOs

- confirm whether Reservit is still active
- confirm current SiteMinder integration
- confirm SiteMinder API access if custom booking UI is desired
- connect contact form
- configure production analytics
- finalize canonical domain strategy
- optimize images to WebP/AVIF
- confirm current legal/privacy/cookie requirements
