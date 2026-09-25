/* ============================================================
   HAVEN — main.js  (vanilla JS, guard-claused, dependency-free)
   ============================================================ */
(function () {
  "use strict";

  var reduceMotion = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

  /* ---------- header: solid on scroll ---------- */
  function initHeader() {
    var header = document.getElementById("siteHeader");
    if (!header) return;
    var toggle = function () {
      header.classList.toggle("scrolled", window.scrollY > 40);
    };
    toggle();
    window.addEventListener("scroll", toggle, { passive: true });
  }

  /* ---------- mobile nav ---------- */
  function initMobileNav() {
    var nav = document.getElementById("mainNav");
    var openBtn = document.getElementById("navToggle");
    var closeBtn = document.getElementById("navClose");
    var overlay = document.getElementById("navOverlay");
    if (!nav || !openBtn) return;

    function open() {
      nav.classList.add("open");
      openBtn.setAttribute("aria-expanded", "true");
      if (overlay) { overlay.hidden = false; requestAnimationFrame(function () { overlay.classList.add("show"); }); }
      document.body.style.overflow = "hidden";
    }
    function close() {
      nav.classList.remove("open");
      openBtn.setAttribute("aria-expanded", "false");
      if (overlay) {
        overlay.classList.remove("show");
        setTimeout(function () { overlay.hidden = true; }, 300);
      }
      document.body.style.overflow = "";
    }

    openBtn.addEventListener("click", open);
    if (closeBtn) closeBtn.addEventListener("click", close);
    if (overlay) overlay.addEventListener("click", close);
    nav.querySelectorAll("a").forEach(function (a) {
      a.addEventListener("click", function () { if (nav.classList.contains("open")) close(); });
    });
    document.addEventListener("keydown", function (e) {
      if (e.key === "Escape" && nav.classList.contains("open")) close();
    });
  }

  /* ---------- fake-submit helper for demo forms ---------- */
  function wireForm(formId, noteId, message) {
    var form = document.getElementById(formId);
    var note = document.getElementById(noteId);
    if (!form) return;
    form.addEventListener("submit", function (e) {
      e.preventDefault();
      if (!form.checkValidity()) { form.reportValidity(); return; }
      if (note) note.textContent = message;
      if (formId !== "bookingForm") form.reset();
    });
  }

  /* ---------- reviews carousel ---------- */
  function initReviews() {
    var track = document.getElementById("rcTrack");
    var viewport = track ? track.parentElement : null;
    var prev = document.getElementById("rcPrev");
    var next = document.getElementById("rcNext");
    var dotsWrap = document.getElementById("rcDots");
    if (!track || !viewport) return;

    var cards = Array.prototype.slice.call(track.children);
    var index = 0;
    var autoTimer = null;

    function perView() {
      var w = window.innerWidth;
      if (w <= 780) return 1;
      if (w <= 1040) return 2;
      return 3;
    }
    function maxIndex() { return Math.max(0, cards.length - perView()); }

    function buildDots() {
      if (!dotsWrap) return;
      dotsWrap.innerHTML = "";
      for (var i = 0; i <= maxIndex(); i++) {
        (function (i) {
          var b = document.createElement("button");
          b.className = "rc-dot" + (i === index ? " active" : "");
          b.type = "button";
          b.setAttribute("aria-label", "Go to review " + (i + 1));
          b.addEventListener("click", function () { go(i); restart(); });
          dotsWrap.appendChild(b);
        })(i);
      }
    }
    function updateDots() {
      if (!dotsWrap) return;
      Array.prototype.forEach.call(dotsWrap.children, function (d, i) {
        d.classList.toggle("active", i === index);
      });
    }
    function go(i) {
      index = Math.max(0, Math.min(i, maxIndex()));
      var target = cards[index];
      track.style.transform = "translateX(" + (-target.offsetLeft) + "px)";
      updateDots();
    }
    function nextSlide() { go(index >= maxIndex() ? 0 : index + 1); }
    function prevSlide() { go(index <= 0 ? maxIndex() : index - 1); }

    function startAuto() {
      if (reduceMotion || cards.length <= perView()) return;
      autoTimer = window.setInterval(nextSlide, 6000);
    }
    function restart() {
      if (autoTimer) { clearInterval(autoTimer); autoTimer = null; }
      startAuto();
    }

    if (next) next.addEventListener("click", function () { nextSlide(); restart(); });
    if (prev) prev.addEventListener("click", function () { prevSlide(); restart(); });
    viewport.addEventListener("mouseenter", function () { if (autoTimer) { clearInterval(autoTimer); autoTimer = null; } });
    viewport.addEventListener("mouseleave", startAuto);

    var rt;
    window.addEventListener("resize", function () {
      clearTimeout(rt);
      rt = setTimeout(function () { buildDots(); go(index); }, 150);
    });

    buildDots();
    go(0);
    startAuto();
  }

  /* ---------- gallery lightbox ---------- */
  function initLightbox() {
    var grid = document.getElementById("galleryGrid");
    var box = document.getElementById("lightbox");
    var img = document.getElementById("lbImg");
    var cap = document.getElementById("lbCaption");
    var closeBtn = document.getElementById("lbClose");
    var prevBtn = document.getElementById("lbPrev");
    var nextBtn = document.getElementById("lbNext");
    if (!grid || !box || !img) return;

    var items = Array.prototype.slice.call(grid.querySelectorAll(".gallery-item"));
    var current = 0;
    var lastFocus = null;

    function show(i) {
      current = (i + items.length) % items.length;
      var el = items[current];
      img.src = el.getAttribute("data-full");
      var c = el.getAttribute("data-caption") || "";
      img.alt = c;
      if (cap) cap.textContent = c;
    }
    function open(i) {
      lastFocus = document.activeElement;
      show(i);
      box.hidden = false;
      box.setAttribute("aria-hidden", "false");
      requestAnimationFrame(function () { box.classList.add("open"); });
      document.body.style.overflow = "hidden";
      if (closeBtn) closeBtn.focus();
    }
    function close() {
      box.classList.remove("open");
      box.setAttribute("aria-hidden", "true");
      setTimeout(function () { box.hidden = true; img.src = ""; }, 300);
      document.body.style.overflow = "";
      if (lastFocus) lastFocus.focus();
    }

    items.forEach(function (el, i) {
      el.addEventListener("click", function () { open(i); });
    });
    if (closeBtn) closeBtn.addEventListener("click", close);
    if (nextBtn) nextBtn.addEventListener("click", function () { show(current + 1); });
    if (prevBtn) prevBtn.addEventListener("click", function () { show(current - 1); });
    box.addEventListener("click", function (e) { if (e.target === box) close(); });
    document.addEventListener("keydown", function (e) {
      if (box.hidden) return;
      if (e.key === "Escape") close();
      else if (e.key === "ArrowRight") show(current + 1);
      else if (e.key === "ArrowLeft") show(current - 1);
    });
  }

  /* ---------- boot ---------- */
  document.addEventListener("DOMContentLoaded", function () {
    initHeader();
    initMobileNav();
    initReviews();
    initLightbox();
    wireForm("bookingForm", "bookingNote", "Thanks — we found 6 suites free for your dates. Our team will email you a booking link shortly.");
    wireForm("enquiryForm", "enquiryNote", "Your enquiry is on its way. We usually reply within a few hours.");
    wireForm("newsForm", "newsNote", "You're on the list. Watch for the next Slow Letter.");
  });
})();
