(() => {
  const toggle = document.querySelector('.menu-toggle');
  const menu = document.querySelector('#primary-menu');
  if (toggle && menu) {
    toggle.addEventListener('click', () => {
      const open = menu.classList.toggle('is-open');
      toggle.setAttribute('aria-expanded', String(open));
    });
  }

  document.querySelectorAll('[data-contact-form]').forEach((form) => {
    form.addEventListener('submit', (event) => {
      event.preventDefault();
      const status = form.querySelector('[data-form-status]');
      if (status) status.textContent = 'Development baseline only: contact delivery is not connected yet.';
      console.warn('TODO: Connect to approved hotel email endpoint/serverless contact handler before production.');
    });
  });
})();
