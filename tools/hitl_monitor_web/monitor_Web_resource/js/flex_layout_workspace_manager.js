/**
 * Provides FlexLayout docking, resizable splitters, and collapsible/maximizable panels.
 */
class FlexLayoutWorkspaceManager {
  /**
   * Initializes splitters, panel header controls, and drag-and-drop docking.
   */
  constructor() {
    this.leftColumn = document.getElementById("colLeft");
    this.rightColumn = document.getElementById("colRight");
    this.draggedPanel = null;
    this.initializeResizableSplitters();
    this.initializeDockablePanels();
  }

  /**
   * Binds pointer drag handlers to the left and right vertical FlexLayout splitters.
   */
  initializeResizableSplitters() {
    const bindSplitter = (splitterId, targetColumn, isLeftColumn) => {
      const splitter = document.getElementById(splitterId);
      if (!splitter || !targetColumn) return;
      let startX = 0;
      let startWidth = 0;
      let isDragging = false;
      splitter.addEventListener("pointerdown", (event) => {
        isDragging = true;
        startX = event.clientX;
        startWidth = targetColumn.getBoundingClientRect().width;
        splitter.classList.add("dragging");
        splitter.setPointerCapture(event.pointerId);
      });
      splitter.addEventListener("pointermove", (event) => {
        if (!isDragging) return;
        const delta = isLeftColumn ? event.clientX - startX : startX - event.clientX;
        targetColumn.style.width = Math.max(240, Math.min(540, startWidth + delta)) + "px";
      });
      const stopDrag = () => {
        isDragging = false;
        splitter.classList.remove("dragging");
      };
      splitter.addEventListener("pointerup", stopDrag);
      splitter.addEventListener("pointercancel", stopDrag);
    };
    bindSplitter("splitLeft", this.leftColumn, true);
    bindSplitter("splitRight", this.rightColumn, false);
  }

  /**
   * Enhances all panels with collapse/maximize buttons and drag-and-drop docking.
   */
  initializeDockablePanels() {
    document.querySelectorAll(".dock-col .pn").forEach((panelElement) => {
      panelElement.setAttribute("draggable", "true");
      const headerElement = panelElement.querySelector("h2");
      if (!headerElement) return;
      const controlsContainer = document.createElement("span");
      controlsContainer.className = "pn-tools";
      controlsContainer.innerHTML =
        '<button type="button" class="pn-btn" data-act="collapse" title="Thu gọn/Mở rộng">−</button>' +
        '<button type="button" class="pn-btn" data-act="max" title="Phóng to/Thu nhỏ">⤢</button>';
      headerElement.appendChild(controlsContainer);

      controlsContainer.querySelector('[data-act="collapse"]').onclick = (event) => {
        event.stopPropagation();
        event.currentTarget.textContent = panelElement.classList.toggle("collapsed") ? "+" : "−";
      };
      controlsContainer.querySelector('[data-act="max"]').onclick = (event) => {
        event.stopPropagation();
        event.currentTarget.textContent = panelElement.classList.toggle("maximized") ? "⤡" : "⤢";
      };
      panelElement.addEventListener("dragstart", () => {
        this.draggedPanel = panelElement;
        panelElement.classList.add("dragging");
      });
      panelElement.addEventListener("dragend", () => {
        panelElement.classList.remove("dragging");
        this.draggedPanel = null;
      });
    });

    document.querySelectorAll(".dock-col").forEach((columnElement) => {
      columnElement.addEventListener("dragover", (event) => {
        event.preventDefault();
        if (!this.draggedPanel) return;
        const afterElement = this.findDropTargetPanel(columnElement, event.clientY);
        afterElement ? columnElement.insertBefore(this.draggedPanel, afterElement) : columnElement.appendChild(this.draggedPanel);
      });
    });
  }

  /**
   * Finds the panel element directly below the current pointer vertical coordinate.
   */
  findDropTargetPanel(columnElement, pointerY) {
    const candidatePanels = [...columnElement.querySelectorAll(".pn:not(.dragging)")];
    return candidatePanels.reduce((closest, child) => {
      const box = child.getBoundingClientRect();
      const offset = pointerY - box.top - box.height / 2;
      return offset < 0 && offset > closest.offset ? { offset, element: child } : closest;
    }, { offset: Number.NEGATIVE_INFINITY, element: null }).element;
  }
}

window.FlexLayoutWorkspaceManager = FlexLayoutWorkspaceManager;
