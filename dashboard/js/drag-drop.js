/**
 * Drag-and-drop system for the FRC 1310 Auto Config sequence editor.
 */

window.DragDrop = (function () {
  'use strict';

  var CP = window.CommandPalette;
  var app = null;

  var dragSource = null;
  var dragCommandType = null;
  var dragStepId = null;
  var dragElement = null;

  var multiSelectIds = new Set();

  function initDragDrop(appRef) {
    app = appRef;
  }

  function setupPaletteDrag(paletteContainer) {
    var items = paletteContainer.querySelectorAll('.palette-item');
    items.forEach(function (item) {
      item.addEventListener('dragstart', handlePaletteDragStart);
      item.addEventListener('dragend', handleDragEnd);
    });
  }

  function handlePaletteDragStart(e) {
    dragSource = 'palette';
    dragCommandType = e.currentTarget.dataset.commandType;
    dragStepId = null;
    dragElement = e.currentTarget;
    e.dataTransfer.effectAllowed = 'copy';
    e.dataTransfer.setData('text/plain', dragCommandType);
    requestAnimationFrame(function () {
      if (dragElement) dragElement.style.opacity = '0.5';
    });
  }

  function setupSequenceDrag(sequenceContainer) {
    var cards = sequenceContainer.querySelectorAll('.step-card');
    cards.forEach(function (card) {
      card.setAttribute('draggable', 'true');
      card.addEventListener('dragstart', handleSequenceDragStart);
      card.addEventListener('dragend', handleDragEnd);
    });
    sequenceContainer.addEventListener('dragover', handleDragOver);
    sequenceContainer.addEventListener('dragleave', handleDragLeave);
    sequenceContainer.addEventListener('drop', handleDrop);
  }

  function handleSequenceDragStart(e) {
    dragSource = 'sequence';
    dragStepId = e.currentTarget.dataset.stepId;
    dragCommandType = null;
    dragElement = e.currentTarget;
    e.dataTransfer.effectAllowed = 'move';
    e.dataTransfer.setData('text/plain', dragStepId);
    requestAnimationFrame(function () {
      if (dragElement) dragElement.classList.add('dragging');
    });
  }

  function handleDragEnd() {
    if (dragElement) {
      dragElement.style.opacity = '';
      dragElement.classList.remove('dragging');
    }
    clearDropIndicators();
    dragSource = null;
    dragCommandType = null;
    dragStepId = null;
    dragElement = null;
  }

  function handleDragOver(e) {
    e.preventDefault();
    e.dataTransfer.dropEffect = dragSource === 'palette' ? 'copy' : 'move';
    var sequenceEl = e.currentTarget;
    clearDropIndicators();
    var dropIndex = getDropIndex(sequenceEl, e.clientY);
    showDropIndicator(sequenceEl, dropIndex);
  }

  function handleDragLeave(e) {
    var sequenceEl = e.currentTarget;
    if (!sequenceEl.contains(e.relatedTarget)) {
      clearDropIndicators();
    }
  }

  function handleDrop(e) {
    e.preventDefault();
    clearDropIndicators();
    if (!app) return;

    var sequenceEl = e.currentTarget;
    var dropIndex = getDropIndex(sequenceEl, e.clientY);

    if (dragSource === 'palette' && dragCommandType) {
      var typeDef = CP.COMMAND_TYPES[dragCommandType];
      if (typeDef) {
        var newStep = Object.assign({}, typeDef.defaultValues, { _id: CP.generateStepId() });
        if (newStep.type === 'parallel' || newStep.type === 'sequential') newStep.commands = [];
        app.insertStep(dropIndex, newStep);
      }
    } else if (dragSource === 'sequence' && dragStepId) {
      var steps = app.getSteps();
      var fromIndex = steps.findIndex(function (s) { return s._id === dragStepId; });
      if (fromIndex !== -1 && fromIndex !== dropIndex && fromIndex !== dropIndex - 1) {
        var adjustedTo = dropIndex > fromIndex ? dropIndex - 1 : dropIndex;
        app.moveStep(fromIndex, adjustedTo);
      }
    }
  }

  function getDropIndex(sequenceEl, clientY) {
    var cards = Array.from(sequenceEl.querySelectorAll('.step-card, .parallel-container, .sequential-container'));
    if (cards.length === 0) return 0;
    for (var i = 0; i < cards.length; i++) {
      var rect = cards[i].getBoundingClientRect();
      if (clientY < rect.top + rect.height / 2) return i;
    }
    return cards.length;
  }

  function showDropIndicator(sequenceEl, index) {
    var indicator = document.createElement('div');
    indicator.className = 'drop-indicator';
    indicator.dataset.dropIndicator = 'true';
    var children = Array.from(sequenceEl.querySelectorAll('.step-card, .parallel-container, .sequential-container'));
    if (index >= children.length) {
      sequenceEl.appendChild(indicator);
    } else {
      sequenceEl.insertBefore(indicator, children[index]);
    }
  }

  function clearDropIndicators() {
    document.querySelectorAll('[data-drop-indicator]').forEach(function (el) { el.remove(); });
  }

  function createStepCard(step, index, isSelected) {
    var typeDef = CP.COMMAND_TYPES[step.type];
    if (!typeDef) return null;
    if (step.type === 'parallel') return createParallelContainer(step, index, isSelected);
    if (step.type === 'sequential') return createSequentialContainer(step, index, isSelected);

    var isMultiSelected = app.isStepMultiSelected && app.isStepMultiSelected(step._id);
    var validationErrors = CP.validateStep(step);
    var isInvalid = validationErrors.length > 0;
    var card = document.createElement('div');
    card.className =
      'step-card' +
      (isSelected ? ' selected' : '') +
      (isMultiSelected ? ' multi-selected' : '') +
      (isInvalid ? ' invalid' : '');
    card.dataset.stepId = step._id;
    card.setAttribute('draggable', 'true');
    var summary = CP.getStepSummary(step);
    if (isInvalid) {
      summary = '\u26a0 ' + validationErrors[0];
    }
    card.innerHTML =
      '<span class="step-number">' + (index + 1) + '</span>' +
      '<span class="step-type-badge ' + typeDef.colorClass + '">' + typeDef.label + '</span>' +
      '<span class="step-summary">' + summary + '</span>' +
      '<span class="step-actions">' +
      '<button class="step-action-btn duplicate-step" title="Duplicate">&#x29C9;</button>' +
      '<button class="step-action-btn delete-step" title="Delete">&#x2715;</button>' +
      '</span>';

    card.addEventListener('click', function (e) {
      if (e.target.closest('.step-action-btn')) return;
      if ((e.ctrlKey || e.metaKey) && app.toggleStepMultiSelect) {
        app.toggleStepMultiSelect(step._id);
        return;
      }
      app.selectStep(step._id);
    });
    card.querySelector('.delete-step').addEventListener('click', function (e) {
      e.stopPropagation();
      app.deleteStep(step._id);
    });
    card.querySelector('.duplicate-step').addEventListener('click', function (e) {
      e.stopPropagation();
      app.duplicateStep(step._id);
    });
    return card;
  }

  function createParallelContainer(step, index, isSelected) {
    var isMultiSelected = app.isStepMultiSelected && app.isStepMultiSelected(step._id);
    var validationErrors = CP.validateStep(step);
    var isInvalid = validationErrors.length > 0;
    var container = document.createElement('div');
    container.className =
      'parallel-container' +
      (isSelected ? ' selected' : '') +
      (isMultiSelected ? ' multi-selected' : '') +
      (isInvalid ? ' invalid' : '');
    container.dataset.stepId = step._id;
    container.setAttribute('draggable', 'true');

    var header = document.createElement('div');
    header.className = 'parallel-header';
    var endConditionText = step.endCondition === 'all'
      ? 'Wait All'
      : (step.endCondition === 'first'
        ? 'Race (First)'
        : ('Deadline #' + (step.deadlineIndex || 1)));
    var timeoutBadge = step.timeoutSeconds > 0
      ? ' <span class="group-timeout-badge">' + step.timeoutSeconds + 's timeout</span>'
      : '';
    header.innerHTML =
      '<span class="parallel-label">' +
      '<span class="step-number">' + (index + 1) + '</span> Parallel Group ' +
      '<span class="parallel-end-condition">' + endConditionText + '</span>' +
      timeoutBadge +
      '</span>' +
      '<span class="step-actions" style="opacity:1;">' +
      '<button class="step-action-btn duplicate-step" title="Duplicate">&#x29C9;</button>' +
      '<button class="step-action-btn delete-step" title="Delete">&#x2715;</button>' +
      '</span>';

    header.addEventListener('click', function (e) {
      if (e.target.closest('.step-action-btn')) return;
      if ((e.ctrlKey || e.metaKey) && app.toggleStepMultiSelect) {
        app.toggleStepMultiSelect(step._id);
        return;
      }
      app.selectStep(step._id);
    });
    header.querySelector('.delete-step').addEventListener('click', function (e) {
      e.stopPropagation();
      app.deleteStep(step._id);
    });
    header.querySelector('.duplicate-step').addEventListener('click', function (e) {
      e.stopPropagation();
      app.duplicateStep(step._id);
    });
    container.appendChild(header);

    var childrenContainer = document.createElement('div');
    childrenContainer.className = 'parallel-children';

    renderGroupChildren(step, childrenContainer, 'Drag commands here or select steps and use "Wrap in Parallel"', ['parallel']);
    container.appendChild(childrenContainer);
    return container;
  }

  function createSequentialContainer(step, index, isSelected) {
    var isMultiSelected = app.isStepMultiSelected && app.isStepMultiSelected(step._id);
    var validationErrors = CP.validateStep(step);
    var isInvalid = validationErrors.length > 0;
    var container = document.createElement('div');
    container.className =
      'sequential-container' +
      (isSelected ? ' selected' : '') +
      (isMultiSelected ? ' multi-selected' : '') +
      (isInvalid ? ' invalid' : '');
    container.dataset.stepId = step._id;
    container.setAttribute('draggable', 'true');

    var header = document.createElement('div');
    header.className = 'sequential-header';
    var seqTimeoutBadge = step.timeoutSeconds > 0
      ? ' <span class="group-timeout-badge">' + step.timeoutSeconds + 's timeout</span>'
      : '';
    header.innerHTML =
      '<span class="sequential-label">' +
      '<span class="step-number">' + (index + 1) + '</span> Sequential Group' +
      seqTimeoutBadge +
      '</span>' +
      '<span class="step-actions" style="opacity:1;">' +
      '<button class="step-action-btn duplicate-step" title="Duplicate">&#x29C9;</button>' +
      '<button class="step-action-btn delete-step" title="Delete">&#x2715;</button>' +
      '</span>';

    header.addEventListener('click', function (e) {
      if (e.target.closest('.step-action-btn')) return;
      if ((e.ctrlKey || e.metaKey) && app.toggleStepMultiSelect) {
        app.toggleStepMultiSelect(step._id);
        return;
      }
      app.selectStep(step._id);
    });
    header.querySelector('.delete-step').addEventListener('click', function (e) {
      e.stopPropagation();
      app.deleteStep(step._id);
    });
    header.querySelector('.duplicate-step').addEventListener('click', function (e) {
      e.stopPropagation();
      app.duplicateStep(step._id);
    });
    container.appendChild(header);

    var childrenContainer = document.createElement('div');
    childrenContainer.className = 'sequential-children';

    renderGroupChildren(step, childrenContainer, 'Drag commands here to build a sequential chain', ['parallel', 'sequential']);
    container.appendChild(childrenContainer);
    return container;
  }

  function renderGroupChildren(step, childrenContainer, emptyText, disallowedDropTypes) {
    if (step.commands && step.commands.length > 0) {
      step.commands.forEach(function (child, ci) {
        // Render nested sequential/parallel as full containers
        if (child.type === 'sequential') {
          var seqContainer = createSequentialContainer(child, ci, app.getSelectedStepId() === child._id);
          if (seqContainer) {
            seqContainer.dataset.parentId = step._id;
            childrenContainer.appendChild(seqContainer);
            return;
          }
        }

        var childTypeDef = CP.COMMAND_TYPES[child.type];
        var childErrors = CP.validateStep(child);
        var childCard = document.createElement('div');
        childCard.className =
          'step-card' +
          (app.getSelectedStepId() === child._id ? ' selected' : '') +
          (childErrors.length > 0 ? ' invalid' : '');
        childCard.dataset.stepId = child._id;
        childCard.dataset.parentId = step._id;
        var childSummary = (childTypeDef ? childTypeDef.summarize(child) : '');
        if (childErrors.length > 0) childSummary = '\u26a0 ' + childErrors[0];
        childCard.innerHTML =
          '<span class="step-number">' + (ci + 1) + '</span>' +
          '<span class="step-type-badge ' + (childTypeDef ? childTypeDef.colorClass : '') + '">' + (childTypeDef ? childTypeDef.label : child.type) + '</span>' +
          '<span class="step-summary">' + childSummary + '</span>' +
          '<span class="step-actions">' +
          '<button class="step-action-btn delete-step" title="Remove from group">&#x2715;</button>' +
          '</span>';

        childCard.addEventListener('click', function (e) {
          if (e.target.closest('.step-action-btn')) return;
          app.selectStep(child._id, step._id);
        });
        childCard.querySelector('.delete-step').addEventListener('click', function (e) {
          e.stopPropagation();
          app.deleteChildStep(step._id, child._id);
        });
        childrenContainer.appendChild(childCard);
      });
    } else {
      var emptyHint = document.createElement('div');
      emptyHint.style.cssText = 'padding:8px; font-size:11px; color:var(--text-muted); text-align:center;';
      emptyHint.textContent = emptyText;
      childrenContainer.appendChild(emptyHint);
    }

    childrenContainer.addEventListener('dragover', function (e) {
      e.preventDefault();
      e.stopPropagation();
      e.dataTransfer.dropEffect = dragSource === 'palette' ? 'copy' : 'move';
      childrenContainer.classList.add('drop-zone-active');
    });
    childrenContainer.addEventListener('dragleave', function (e) {
      if (!childrenContainer.contains(e.relatedTarget)) {
        childrenContainer.classList.remove('drop-zone-active');
      }
    });
    childrenContainer.addEventListener('drop', function (e) {
      e.preventDefault();
      e.stopPropagation();
      childrenContainer.classList.remove('drop-zone-active');

      if (dragSource === 'palette' && dragCommandType) {
        var td = CP.COMMAND_TYPES[dragCommandType];
        if (td && disallowedDropTypes.indexOf(dragCommandType) === -1) {
          var newChild = Object.assign({}, td.defaultValues, { _id: CP.generateStepId() });
          app.addChildToParallel(step._id, newChild);
        }
      } else if (dragSource === 'sequence' && dragStepId) {
        app.moveStepIntoParallel(dragStepId, step._id);
      }
    });
  }

  function toggleMultiSelect(stepId) {
    if (multiSelectIds.has(stepId)) multiSelectIds.delete(stepId);
    else multiSelectIds.add(stepId);
    return new Set(multiSelectIds);
  }

  function clearMultiSelect() { multiSelectIds.clear(); }
  function getMultiSelectIds() { return new Set(multiSelectIds); }

  function wrapSelectedInParallel() {
    if (multiSelectIds.size < 2) return;
    app.wrapInParallel(new Set(multiSelectIds));
    multiSelectIds.clear();
  }

  return {
    initDragDrop: initDragDrop,
    setupPaletteDrag: setupPaletteDrag,
    setupSequenceDrag: setupSequenceDrag,
    createStepCard: createStepCard,
    toggleMultiSelect: toggleMultiSelect,
    clearMultiSelect: clearMultiSelect,
    getMultiSelectIds: getMultiSelectIds,
    wrapSelectedInParallel: wrapSelectedInParallel,
  };
})();
