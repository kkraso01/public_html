/**
 * Workspace Tabs System
 * Browser/IDE-style vertical tabs for organizing portfolio content
 * Preserves neon-lab aesthetic with smooth transitions
 */

(function () {
  const STORAGE_KEY = 'portfolioActiveWorkspace';
  const WORKSPACE_CLASS = 'workspace-active';
  const HIDDEN_CLASS = 'workspace-hidden';

  const workspaces = ['llm', 'autonomy', 'profile'];
  let activeWorkspace = localStorage.getItem(STORAGE_KEY) || 'llm';

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

    // Scroll to top of new workspace smoothly
    window.scrollTo({ top: 0, behavior: 'smooth' });
  }

  // Switch to a different workspace
  function switchWorkspace(workspace) {
    applyWorkspace(workspace);
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
