/**
 * Workspace Tabs System
 * Browser-style horizontal tabs for organizing portfolio content
 * Positioned below hero, with keyboard navigation support
 */

(function () {
  const STORAGE_KEY = 'portfolioActiveWorkspace';
  const WORKSPACE_CLASS = 'workspace-active';
  const HIDDEN_CLASS = 'workspace-hidden';

  const workspaces = ['llm', 'autonomy', 'profile'];
  let activeWorkspace = localStorage.getItem(STORAGE_KEY) || 'profile';

  // Get all sections grouped by workspace
  function getSectionsByWorkspace(workspace) {
    return document.querySelectorAll(`[data-workspace="${workspace}"]`);
  }

  // Apply active workspace state
  function applyWorkspace(workspace) {
    if (!workspaces.includes(workspace)) {
      workspace = 'llm';
    }

    activeWorkspace = workspace;
    localStorage.setItem(STORAGE_KEY, workspace);

    // Update body data attribute for potential canvas guards
    document.body.dataset.workspace = workspace;

    // Update tab button states
    document.querySelectorAll('[data-tab]').forEach((tab) => {
      const isActive = tab.getAttribute('data-tab') === workspace;
      tab.classList.toggle('active', isActive);
      tab.setAttribute('aria-selected', isActive);
    });

    // Show/hide sections based on workspace
    workspaces.forEach((ws) => {
      const sections = getSectionsByWorkspace(ws);
      const isVisible = ws === workspace;

      sections.forEach((section) => {
        // Skip hero section - it's always visible
        if (section.hasAttribute('data-workspace-shared')) {
          return;
        }

        if (isVisible) {
          section.classList.remove(HIDDEN_CLASS);
          section.classList.add(WORKSPACE_CLASS);
        } else {
          section.classList.add(HIDDEN_CLASS);
          section.classList.remove(WORKSPACE_CLASS);
        }
      });
    });

    // Force resize event for canvas animations
    setTimeout(() => {
      window.dispatchEvent(new Event('resize'));
    }, 100);

    // Scroll to position tabs at top of viewport
    setTimeout(() => {
      const tabBar = document.querySelector('.workspace-tabs-container');
      if (tabBar) {
        const tabTop = tabBar.getBoundingClientRect().top + window.scrollY;
        window.scrollTo({ top: tabTop - 10, behavior: 'smooth' });
      }
    }, 150);
  }

  // Switch to a different workspace
  function switchWorkspace(workspace) {
    applyWorkspace(workspace);
  }

  // Keyboard navigation (Arrow Left/Right)
  function handleKeyboardNav(e) {
    if (e.key !== 'ArrowLeft' && e.key !== 'ArrowRight') return;
    
    // Don't interfere with input fields
    const activeTag = document.activeElement?.tagName?.toLowerCase();
    if (activeTag === 'input' || activeTag === 'textarea') return;

    const currentIndex = workspaces.indexOf(activeWorkspace);
    let newIndex;

    if (e.key === 'ArrowLeft') {
      newIndex = currentIndex > 0 ? currentIndex - 1 : workspaces.length - 1;
    } else {
      newIndex = currentIndex < workspaces.length - 1 ? currentIndex + 1 : 0;
    }

    switchWorkspace(workspaces[newIndex]);
    
    // Focus the newly active tab
    const activeTab = document.querySelector(`[data-tab="${workspaces[newIndex]}"]`);
    if (activeTab) activeTab.focus();
  }

  // Initialize on DOM ready
  function init() {
    // Attach click handlers to tab buttons
    document.querySelectorAll('[data-tab]').forEach((tab) => {
      tab.addEventListener('click', (e) => {
        e.preventDefault();
        const workspace = tab.getAttribute('data-tab');
        switchWorkspace(workspace);
      });
    });

    // Add keyboard navigation
    document.addEventListener('keydown', handleKeyboardNav);

    // Apply initial workspace
    applyWorkspace(activeWorkspace);
  }

  // Wait for DOM
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', init);
  } else {
    init();
  }

  // Expose API for debugging
  window.WorkspaceTabs = {
    switch: switchWorkspace,
    active: () => activeWorkspace,
    list: () => workspaces,
  };
})();
