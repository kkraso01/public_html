/* Booking integrations are intentionally isolated for later replacement. */
document.querySelectorAll('[data-booking-widget]').forEach((frame) => {
  frame.setAttribute('title', frame.dataset.bookingTitle || 'Online booking');
});

/* TODO: Confirm with hotel whether Reservit hotelid=172781 is still active before production launch. */
