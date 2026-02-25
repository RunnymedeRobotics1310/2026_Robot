/**
 * FRC Team 1310 Auto Configuration Dashboard — Main Application
 */

(function () {
  'use strict';

  var CP = window.CommandPalette;
  var DD = window.DragDrop;
  var NT = window.NTBridge;

  // ===== Application State =====

  // Cache of received configs so clicking loads instantly
  var configCache = {}; // name -> configJson

  // Locally-created auto names (not yet on robot)
  var localAutoNames = [];

  var state = {
    autoNames: [],
    deployAutoNames: [],
    runtimeAutoNames: [],
    currentConfig: null,
    steps: [],
    selectedStepId: null,
    selectedParentId: null,
    connected: false,
    autoConfigSupported: null, // null=unknown, true=supported, false=unsupported
  };

  var connectionMessage = 'Disconnected';

  var isDirty = false;
  var history = [];
  var historyIndex = -1;
  var savedHistoryIndex = -1;
  var restoringHistory = false;
  var HISTORY_LIMIT = 80;

  var robotSyncedHashesByName = {};
  var repoSyncedHashesByName = {};
  var pendingRobotSync = null;
  var repoAutosDirHandle = null;
  var PANEL_LAYOUT_KEY = '1310-autoconfig-panel-layout-v1';
  var PANEL_LEFT_MIN_PX = 200;
  var PANEL_RIGHT_MIN_PX = 240;
  var PANEL_CENTER_MIN_PX = 320;

  // ===== DOM References =====

  var dom = {};

  function cacheDom() {
    dom = {
      statusDot: document.getElementById('status-dot'),
      statusText: document.getElementById('status-text'),
      teamNumber: document.getElementById('team-number'),
      mainLayout: document.getElementById('main-layout'),
      panelLeft: document.getElementById('panel-left'),
      panelCenter: document.getElementById('panel-center'),
      panelRight: document.getElementById('panel-right'),
      splitterLeft: document.getElementById('splitter-left'),
      splitterRight: document.getElementById('splitter-right'),
      connectBtn: document.getElementById('connect-btn'),
      saveBtn: document.getElementById('save-btn'),
      saveRepoBtn: document.getElementById('save-repo-btn'),
      localFolderBtn: document.getElementById('local-folder-btn'),
      exportBtn: document.getElementById('export-btn'),
      importBtn: document.getElementById('import-btn'),
      importFile: document.getElementById('import-file'),
      localFolderInput: document.getElementById('local-folder-input'),
      editStatus: document.getElementById('edit-status'),
      writeStatus: document.getElementById('write-status'),
      autoList: document.getElementById('auto-list'),
      autoFilter: document.getElementById('auto-filter'),
      newAutoName: document.getElementById('new-auto-name'),
      newAutoBtn: document.getElementById('new-auto-btn'),
      palette: document.getElementById('palette'),
      sequenceStats: document.getElementById('sequence-stats'),
      sequenceEditor: document.getElementById('sequence-editor'),
      undoBtn: document.getElementById('undo-btn'),
      redoBtn: document.getElementById('redo-btn'),
      moveUpBtn: document.getElementById('move-up-btn'),
      moveDownBtn: document.getElementById('move-down-btn'),
      duplicateSelectedBtn: document.getElementById('duplicate-selected-btn'),
      deleteSelectedBtn: document.getElementById('delete-selected-btn'),
      clearSequenceBtn: document.getElementById('clear-sequence-btn'),
      validateBtn: document.getElementById('validate-btn'),
      wrapToolbar: document.getElementById('wrap-toolbar'),
      wrapBtn: document.getElementById('wrap-btn'),
      selectionInfo: document.getElementById('selection-info'),
      configName: document.getElementById('config-name'),
      configDescription: document.getElementById('config-description'),
      configHeading: document.getElementById('config-heading'),
      propertiesContainer: document.getElementById('properties-container'),
    };
  }

  // ===== Team Number → IP =====

  function teamNumberToIP(teamStr) {
    var num = parseInt(teamStr, 10);
    if (isNaN(num) || num < 1 || num > 99999) return null;
    if (num <= 9999) {
      // 4-digit (or fewer): 10.TE.AM.2
      var te = Math.floor(num / 100);
      var am = num % 100;
      return '10.' + te + '.' + am + '.2';
    }
    // 5-digit: 10.TEA.MS.2
    var tea = Math.floor(num / 100);
    var ms = num % 100;
    return '10.' + tea + '.' + ms + '.2';
  }

  var STORAGE_KEY = '1310-autoconfig-team';

  function sanitizeAutoName(name) {
    if (!name) return '';
    return String(name)
      .trim()
      .toLowerCase()
      .replace(/[^a-z0-9_]/g, '_')
      .replace(/_+/g, '_')
      .replace(/^_+|_+$/g, '')
      .slice(0, 40);
  }

  function deepClone(value) {
    return JSON.parse(JSON.stringify(value));
  }

  function clampNumber(value, min, max) {
    return Math.max(min, Math.min(max, value));
  }

  function isStackedLayout() {
    return window.matchMedia('(max-width: 900px)').matches;
  }

  function getPanelWidth(panelEl) {
    return panelEl ? panelEl.getBoundingClientRect().width : 0;
  }

  function setPanelWidth(side, widthPx) {
    if (!dom.mainLayout) return;
    var varName = side === 'left' ? '--panel-left-width' : '--panel-right-width';
    dom.mainLayout.style.setProperty(varName, Math.round(widthPx) + 'px');
  }

  function readSavedPanelLayout() {
    try {
      var raw = localStorage.getItem(PANEL_LAYOUT_KEY);
      if (!raw) return null;
      var parsed = JSON.parse(raw);
      if (!parsed || typeof parsed !== 'object') return null;
      return parsed;
    } catch (err) {
      return null;
    }
  }

  function savePanelLayout() {
    if (!dom.mainLayout || isStackedLayout()) return;
    var payload = {
      left: Math.round(getPanelWidth(dom.panelLeft)),
      right: Math.round(getPanelWidth(dom.panelRight)),
    };
    try {
      localStorage.setItem(PANEL_LAYOUT_KEY, JSON.stringify(payload));
    } catch (err) {
      // Ignore storage errors.
    }
  }

  function normalizePanelLayout() {
    if (!dom.mainLayout || !dom.panelLeft || !dom.panelRight || !dom.panelCenter) return;
    if (isStackedLayout()) return;

    var leftWidth = Math.max(PANEL_LEFT_MIN_PX, getPanelWidth(dom.panelLeft));
    var rightWidth = Math.max(PANEL_RIGHT_MIN_PX, getPanelWidth(dom.panelRight));
    setPanelWidth('left', leftWidth);
    setPanelWidth('right', rightWidth);

    var centerWidth = getPanelWidth(dom.panelCenter);
    if (centerWidth >= PANEL_CENTER_MIN_PX) return;

    var deficit = PANEL_CENTER_MIN_PX - centerWidth;
    var leftHeadroom = Math.max(0, leftWidth - PANEL_LEFT_MIN_PX);
    var rightHeadroom = Math.max(0, rightWidth - PANEL_RIGHT_MIN_PX);

    var takeLeft = Math.min(leftHeadroom, Math.ceil(deficit / 2));
    leftWidth -= takeLeft;
    deficit -= takeLeft;

    var takeRight = Math.min(rightHeadroom, deficit);
    rightWidth -= takeRight;
    deficit -= takeRight;

    if (deficit > 0) {
      var extraLeft = Math.min(Math.max(0, leftWidth - PANEL_LEFT_MIN_PX), deficit);
      leftWidth -= extraLeft;
      deficit -= extraLeft;
    }
    if (deficit > 0) {
      var extraRight = Math.min(Math.max(0, rightWidth - PANEL_RIGHT_MIN_PX), deficit);
      rightWidth -= extraRight;
    }

    setPanelWidth('left', leftWidth);
    setPanelWidth('right', rightWidth);
  }

  function startPanelResize(side, event) {
    if (isStackedLayout()) return;
    event.preventDefault();

    var startX = event.clientX;
    var startWidth = side === 'left' ? getPanelWidth(dom.panelLeft) : getPanelWidth(dom.panelRight);
    var startCenterWidth = getPanelWidth(dom.panelCenter);
    var minWidth = side === 'left' ? PANEL_LEFT_MIN_PX : PANEL_RIGHT_MIN_PX;
    var maxWidth = startWidth + Math.max(0, startCenterWidth - PANEL_CENTER_MIN_PX);
    var splitter = side === 'left' ? dom.splitterLeft : dom.splitterRight;

    splitter.classList.add('dragging');
    document.body.classList.add('resizing-panels');

    function onMouseMove(moveEvent) {
      var delta = moveEvent.clientX - startX;
      var proposed = side === 'left' ? startWidth + delta : startWidth - delta;
      var nextWidth = clampNumber(proposed, minWidth, maxWidth);
      setPanelWidth(side, nextWidth);
    }

    function onMouseUp() {
      document.removeEventListener('mousemove', onMouseMove);
      document.removeEventListener('mouseup', onMouseUp);
      splitter.classList.remove('dragging');
      document.body.classList.remove('resizing-panels');
      normalizePanelLayout();
      savePanelLayout();
    }

    document.addEventListener('mousemove', onMouseMove);
    document.addEventListener('mouseup', onMouseUp);
  }

  function initPanelResizers() {
    if (!dom.mainLayout || !dom.splitterLeft || !dom.splitterRight) return;

    var saved = readSavedPanelLayout();
    if (saved) {
      if (typeof saved.left === 'number' && isFinite(saved.left)) {
        setPanelWidth('left', saved.left);
      }
      if (typeof saved.right === 'number' && isFinite(saved.right)) {
        setPanelWidth('right', saved.right);
      }
    }
    normalizePanelLayout();

    dom.splitterLeft.addEventListener('mousedown', function (e) {
      startPanelResize('left', e);
    });
    dom.splitterRight.addEventListener('mousedown', function (e) {
      startPanelResize('right', e);
    });

    window.addEventListener('resize', function () {
      normalizePanelLayout();
    });
  }

  function snapshotState() {
    return {
      currentConfig: state.currentConfig ? deepClone(state.currentConfig) : null,
      steps: deepClone(state.steps),
    };
  }

  function snapshotsEqual(a, b) {
    return JSON.stringify(a) === JSON.stringify(b);
  }

  function restoreSnapshot(snapshot) {
    restoringHistory = true;
    state.currentConfig = snapshot.currentConfig ? deepClone(snapshot.currentConfig) : null;
    state.steps = deepClone(snapshot.steps || []);
    state.selectedStepId = null;
    state.selectedParentId = null;
    DD.clearMultiSelect();
    syncConfigFields();
    renderAutoList();
    renderSequence();
    renderProperties();
    restoringHistory = false;
  }

  function pushHistory() {
    if (restoringHistory) return;
    var snap = snapshotState();
    if (historyIndex >= 0 && snapshotsEqual(history[historyIndex], snap)) return;

    history = history.slice(0, historyIndex + 1);
    history.push(snap);
    if (history.length > HISTORY_LIMIT) {
      history.shift();
    }
    historyIndex = history.length - 1;
    updateActionButtons();
  }

  function undoHistory() {
    if (historyIndex <= 0) return;
    historyIndex--;
    restoreSnapshot(history[historyIndex]);
    setDirty(savedHistoryIndex < 0 || historyIndex !== savedHistoryIndex);
    updateActionButtons();
  }

  function redoHistory() {
    if (historyIndex >= history.length - 1) return;
    historyIndex++;
    restoreSnapshot(history[historyIndex]);
    setDirty(savedHistoryIndex < 0 || historyIndex !== savedHistoryIndex);
    updateActionButtons();
  }

  function setDirty(dirty) {
    isDirty = !!dirty;
    updateEditStatus();
    updateActionButtons();
  }

  function markDirty() {
    if (restoringHistory) return;
    pushHistory();
    setDirty(savedHistoryIndex < 0 || historyIndex !== savedHistoryIndex);
  }

  function updateEditStatus() {
    if (!dom.editStatus) return;
    if (!state.currentConfig) {
      dom.editStatus.className = 'status-message';
      dom.editStatus.textContent = 'No auto loaded';
      return;
    }

    var name = sanitizeAutoName(state.currentConfig.name || '');
    var currentHash = currentConfigHash();
    var robotHash = name ? robotSyncedHashesByName[name] : null;
    var repoHash = name ? repoSyncedHashesByName[name] : null;
    var syncedToRobot = !!currentHash && currentHash === robotHash;
    var syncedToRepo = !!currentHash && currentHash === repoHash;

    if (isDirty) {
      dom.editStatus.className = 'status-message dirty';
      if (syncedToRobot && syncedToRepo) {
        dom.editStatus.textContent = 'Edited since robot + repo sync';
      } else if (syncedToRobot) {
        dom.editStatus.textContent = 'Edited since robot sync';
      } else if (syncedToRepo) {
        dom.editStatus.textContent = 'Edited since repo sync';
      } else {
        dom.editStatus.textContent = 'Not synced to robot or repo';
      }
      return;
    }

    if (syncedToRobot && syncedToRepo) {
      dom.editStatus.className = 'status-message success';
      dom.editStatus.textContent = 'Synced to robot + repo';
      return;
    }
    if (syncedToRobot) {
      dom.editStatus.className = 'status-message success';
      dom.editStatus.textContent = 'Synced to robot runtime only';
      return;
    }
    if (syncedToRepo) {
      dom.editStatus.className = 'status-message dirty';
      dom.editStatus.textContent = 'Synced to repo only';
      return;
    }

    dom.editStatus.className = 'status-message dirty';
    dom.editStatus.textContent = 'Not synced to robot or repo';
  }

  function confirmDiscardChanges(actionText) {
    if (!isDirty) return true;
    return window.confirm('You have unsaved changes. ' + actionText + ' and discard changes?');
  }

  function syncConfigFields() {
    if (!state.currentConfig) {
      dom.configName.value = '';
      dom.configDescription.value = '';
      dom.configHeading.value = '0';
      return;
    }
    dom.configName.value = state.currentConfig.name || '';
    dom.configDescription.value = state.currentConfig.description || '';
    dom.configHeading.value = String(state.currentConfig.startingHeadingDegrees || 0);
  }

  function hashConfig(config) {
    var copy = deepClone(config || {});
    if (copy.name) {
      copy.name = sanitizeAutoName(copy.name);
    }
    return JSON.stringify(copy);
  }

  function currentConfigHash() {
    var config = serializeConfig();
    if (!config) return null;
    return hashConfig(config);
  }

  function isValidConfigShape(configJson) {
    return !!(configJson && typeof configJson === 'object' && Array.isArray(configJson.steps));
  }

  function addLocalConfig(configJson) {
    if (!isValidConfigShape(configJson)) return false;
    var safeName = sanitizeAutoName(configJson.name || '');
    if (!safeName) return false;

    var normalized = deepClone(configJson);
    normalized.name = safeName;

    configCache[safeName] = normalized;
    repoSyncedHashesByName[safeName] = hashConfig(normalized);

    if (state.autoNames.indexOf(safeName) === -1 && localAutoNames.indexOf(safeName) === -1) {
      localAutoNames.push(safeName);
    }
    return true;
  }

  // ===== Initialization =====

  document.addEventListener('DOMContentLoaded', function () {
    cacheDom();

    // Restore last team number from localStorage
    var saved = localStorage.getItem(STORAGE_KEY);
    if (saved) dom.teamNumber.value = saved;

    initEventListeners();
    initPanelResizers();

    CP.createPaletteItems(dom.palette);
    DD.setupPaletteDrag(dom.palette);
    DD.initDragDrop(appInterface);

    NT.init({
      onConnectionChange: handleConnectionChange,
      onAutoListUpdate: handleAutoListUpdate,
      onDeployAutoListUpdate: handleDeployAutoListUpdate,
      onRuntimeAutoListUpdate: handleRuntimeAutoListUpdate,
      onConfigReceived: handleConfigReceived,
      onWriteStatus: handleWriteStatus,
      onFeatureSupportChange: handleFeatureSupportChange,
    });

    renderSequence();
    renderAutoList();
    renderProperties();
    updateEditStatus();
    updateConnectionStatus();
    updateActionButtons();

    window.addEventListener('beforeunload', function (event) {
      if (!isDirty) return;
      event.preventDefault();
      event.returnValue = '';
    });
  });

  // ===== Event Listeners =====

  function initEventListeners() {
    dom.connectBtn.addEventListener('click', function () {
      if (state.connected) {
        NT.disconnect();
      } else {
        var teamStr = dom.teamNumber.value.trim() || '1310';
        var ip = teamNumberToIP(teamStr);
        if (!ip) {
          showToast('Invalid team number: ' + teamStr, 'error');
          return;
        }
        localStorage.setItem(STORAGE_KEY, teamStr);
        NT.connect(ip);
      }
    });

    dom.saveBtn.addEventListener('click', saveToRobot);
    dom.saveRepoBtn.addEventListener('click', saveToRepoFile);
    dom.localFolderBtn.addEventListener('click', function () { dom.localFolderInput.click(); });
    dom.exportBtn.addEventListener('click', exportToFile);
    dom.importBtn.addEventListener('click', function () { dom.importFile.click(); });
    dom.importFile.addEventListener('change', importFromFile);
    dom.localFolderInput.addEventListener('change', importLocalFolderFiles);
    dom.newAutoBtn.addEventListener('click', createNewAuto);
    dom.newAutoName.addEventListener('keydown', function (e) {
      if (e.key === 'Enter') createNewAuto();
    });
    dom.autoFilter.addEventListener('input', renderAutoList);

    dom.configName.addEventListener('input', function () {
      if (!state.currentConfig) return;
      state.currentConfig.name = dom.configName.value;
      setDirty(true);
      renderAutoList();
    });
    dom.configName.addEventListener('change', function () {
      if (!state.currentConfig) return;
      state.currentConfig.name = dom.configName.value;
      markDirty();
    });
    dom.configDescription.addEventListener('input', function () {
      if (!state.currentConfig) return;
      state.currentConfig.description = dom.configDescription.value;
      setDirty(true);
    });
    dom.configDescription.addEventListener('change', function () {
      if (!state.currentConfig) return;
      state.currentConfig.description = dom.configDescription.value;
      markDirty();
    });
    dom.configHeading.addEventListener('change', function () {
      if (!state.currentConfig) return;
      state.currentConfig.startingHeadingDegrees = parseFloat(dom.configHeading.value) || 0;
      markDirty();
    });

    dom.undoBtn.addEventListener('click', undoHistory);
    dom.redoBtn.addEventListener('click', redoHistory);
    dom.moveUpBtn.addEventListener('click', function () { moveSelectedStep(-1); });
    dom.moveDownBtn.addEventListener('click', function () { moveSelectedStep(1); });
    dom.duplicateSelectedBtn.addEventListener('click', function () {
      if (state.selectedStepId) duplicateStep(state.selectedStepId);
    });
    dom.deleteSelectedBtn.addEventListener('click', function () {
      if (state.selectedStepId) deleteStep(state.selectedStepId);
    });
    dom.clearSequenceBtn.addEventListener('click', clearSequence);
    dom.validateBtn.addEventListener('click', validateCurrentConfig);

    dom.wrapBtn.addEventListener('click', function () {
      DD.wrapSelectedInParallel();
      renderSequence();
      renderProperties();
      updateWrapToolbar();
    });

    document.addEventListener('keydown', function (e) {
      if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 's') {
        e.preventDefault();
        saveToRobot();
        return;
      }
      if ((e.ctrlKey || e.metaKey) && !e.shiftKey && e.key.toLowerCase() === 'z') {
        e.preventDefault();
        undoHistory();
        return;
      }
      if ((e.ctrlKey || e.metaKey) && e.shiftKey && e.key.toLowerCase() === 'z') {
        e.preventDefault();
        redoHistory();
        return;
      }

      if (e.key === 'Delete' || e.key === 'Backspace') {
        if (document.activeElement.tagName !== 'INPUT' &&
            document.activeElement.tagName !== 'TEXTAREA' &&
            document.activeElement.tagName !== 'SELECT') {
          if (state.selectedStepId) deleteStep(state.selectedStepId);
        }
      }
      if (e.key === 'Escape') {
        state.selectedStepId = null;
        state.selectedParentId = null;
        DD.clearMultiSelect();
        renderSequence();
        renderProperties();
        updateWrapToolbar();
      }

      if (document.activeElement.tagName === 'INPUT' ||
          document.activeElement.tagName === 'TEXTAREA' ||
          document.activeElement.tagName === 'SELECT') {
        return;
      }
      if (e.key === 'ArrowUp') {
        e.preventDefault();
        moveSelectedStep(-1);
      } else if (e.key === 'ArrowDown') {
        e.preventDefault();
        moveSelectedStep(1);
      } else if (e.key.toLowerCase() === 'd' && (e.ctrlKey || e.metaKey)) {
        e.preventDefault();
        if (state.selectedStepId) duplicateStep(state.selectedStepId);
      }
    });

    // Palette click-to-add
    var paletteWasDragged = false;
    document.addEventListener('dragstart', function (e) {
      if (e.target.closest && e.target.closest('.palette-item')) paletteWasDragged = true;
    });
    document.addEventListener('dragend', function () {
      setTimeout(function () { paletteWasDragged = false; }, 100);
    });
    document.addEventListener('click', function (e) {
      if (paletteWasDragged) return;
      var paletteItem = e.target.closest ? e.target.closest('.palette-item') : null;
      if (paletteItem) {
        var typeKey = paletteItem.dataset.commandType;
        var typeDef = CP.COMMAND_TYPES[typeKey];
        if (typeDef && state.currentConfig) {
          var newStep = Object.assign({}, typeDef.defaultValues, { _id: CP.generateStepId() });
          if (newStep.type === 'parallel') newStep.commands = [];
          state.steps.push(newStep);
          state.selectedStepId = newStep._id;
          state.selectedParentId = null;
          markDirty();
          renderSequence();
          renderProperties();
        }
      }
    });
  }

  // ===== NT Bridge Callbacks =====

  function handleConnectionChange(connected, message) {
    state.connected = connected;
    connectionMessage = message || (connected ? 'Connected' : 'Disconnected');
    if (!connected) {
      state.autoConfigSupported = null;
      state.autoNames = [];
      state.deployAutoNames = [];
      state.runtimeAutoNames = [];
      pendingLoadName = null;
    }
    dom.statusDot.className = 'status-dot' + (connected ? ' connected' : '');
    updateConnectionStatus();
    dom.connectBtn.textContent = connected ? 'Disconnect' : 'Connect';
    dom.connectBtn.className = connected ? 'btn btn-secondary' : 'btn btn-primary';
    renderAutoList();
    updateActionButtons();
  }

  function handleFeatureSupportChange(supported, message) {
    state.autoConfigSupported = supported;
    if (supported === false && message) {
      connectionMessage = message;
    } else if (supported === true) {
      connectionMessage = 'Connected';
    } else if (supported === null && message) {
      connectionMessage = message;
    }
    updateConnectionStatus();
    if (state.connected && supported === false) {
      showToast('Connected robot does not support this auto-config dashboard bridge', 'error');
    }
    updateActionButtons();
    renderAutoList();
  }

  function updateConnectionStatus() {
    if (!dom.statusText) return;

    if (!state.connected) {
      dom.statusText.className = 'status-message';
      dom.statusText.textContent = connectionMessage || 'Disconnected';
      return;
    }

    if (state.autoConfigSupported === false) {
      dom.statusText.className = 'status-message error';
      dom.statusText.textContent = 'Connected - auto-config unsupported on robot';
      return;
    }

    if (state.autoConfigSupported === null) {
      dom.statusText.className = 'status-message';
      dom.statusText.textContent = 'Connected - checking auto-config support...';
      return;
    }

    dom.statusText.className = 'status-message success';
    dom.statusText.textContent = connectionMessage || 'Connected';
  }

  function handleAutoListUpdate(names) {
    state.autoNames = names;
    // Remove local names that now exist on the robot (they've been saved)
    localAutoNames = localAutoNames.filter(function (n) { return names.indexOf(n) === -1; });
    renderAutoList();
    // Pre-fetch all configs so cache stays fresh.
    for (var i = 0; i < names.length; i++) {
      NT.requestConfig(names[i]);
    }
  }

  function handleRuntimeAutoListUpdate(names) {
    state.runtimeAutoNames = Array.isArray(names) ? names.slice() : [];
    renderAutoList();
  }

  function handleDeployAutoListUpdate(names) {
    state.deployAutoNames = Array.isArray(names) ? names.slice() : [];
    renderAutoList();
  }

  // Name of the config the user explicitly asked to load (via click)
  var pendingLoadName = null;

  function handleConfigReceived(name, configJson) {
    configCache[name] = configJson;
    robotSyncedHashesByName[name] = hashConfig(configJson);
    updateEditStatus();
    // Load into editor if the user explicitly requested this config.
    if (pendingLoadName === name) {
      pendingLoadName = null;
      loadConfig(configJson);
      showToast('Loaded "' + name + '" from robot', 'success');
      return;
    }

    // Keep the editor fresh if this config is currently open.
    if (state.currentConfig && state.currentConfig.name === name) {
      if (isDirty) {
        return;
      }
      loadConfig(configJson);
    }
  }

  function handleWriteStatus(status) {
    dom.writeStatus.textContent = status;
    var isOk = (typeof status === 'string') && status.indexOf('ok') === 0;
    dom.writeStatus.className = 'status-message ' + (isOk ? 'success' : 'error');
    if (isOk) {
      if (pendingRobotSync && pendingRobotSync.name) {
        robotSyncedHashesByName[pendingRobotSync.name] = pendingRobotSync.hash;
      }
      pendingRobotSync = null;
      pushHistory();
      savedHistoryIndex = historyIndex;
      setDirty(false);
      updateActionButtons();
      if (String(status).indexOf('runtime') !== -1) {
        showToast('Saved to robot (runtime only)', 'success');
      } else {
        showToast('Saved to robot successfully', 'success');
      }
    }
    else if (status) {
      pendingRobotSync = null;
      showToast('Robot: ' + status, 'error');
    }
  }

  // ===== Auto List Management =====

  function renderAutoList() {
    dom.autoList.innerHTML = '';
    // Merge robot names with locally-created names (deduplicated)
    var nameSet = {};
    var allNames = [];
    var deploySet = {};
    var runtimeSet = {};
    var localSet = {};
    state.autoNames.forEach(function (n) { if (!nameSet[n]) { nameSet[n] = true; allNames.push(n); } });
    state.deployAutoNames.forEach(function (n) { deploySet[n] = true; });
    state.runtimeAutoNames.forEach(function (n) { runtimeSet[n] = true; });
    localAutoNames.forEach(function (n) { if (!nameSet[n]) { nameSet[n] = true; allNames.push(n); } });
    localAutoNames.forEach(function (n) { localSet[n] = true; });
    if (state.currentConfig && state.currentConfig.name && !nameSet[state.currentConfig.name]) {
      allNames.push(state.currentConfig.name);
      localSet[state.currentConfig.name] = true;
    }

    allNames.sort(function (a, b) { return a.localeCompare(b); });
    var filterText = (dom.autoFilter.value || '').trim().toLowerCase();
    if (filterText) {
      allNames = allNames.filter(function (n) { return n.toLowerCase().indexOf(filterText) !== -1; });
    }

    if (allNames.length === 0) {
      var empty = document.createElement('div');
      empty.style.cssText = 'padding:8px; font-size:12px; color:var(--text-muted); text-align:center;';
      if (filterText) {
        empty.textContent = 'No autos match filter.';
      } else if (state.connected && state.autoConfigSupported === false) {
        empty.textContent = 'Robot connected, but this build does not support the auto-config NT bridge.';
      } else {
        empty.textContent = 'No autos loaded. Create one or connect to robot.';
      }
      dom.autoList.appendChild(empty);
      return;
    }

    for (var i = 0; i < allNames.length; i++) {
      (function (name) {
        var item = document.createElement('div');
        item.className = 'auto-list-item' +
          (state.currentConfig && state.currentConfig.name === name ? ' active' : '');

        var nameSpan = document.createElement('span');
        nameSpan.className = 'auto-list-name';
        nameSpan.textContent = name;
        item.appendChild(nameSpan);

        var sourceBadge = document.createElement('span');
        if (runtimeSet[name] && deploySet[name]) {
          sourceBadge.className = 'auto-source-badge runtime-override';
          sourceBadge.textContent = 'runtime+deploy';
          sourceBadge.title = 'Runtime override active (deploy baseline also exists)';
        } else if (runtimeSet[name]) {
          sourceBadge.className = 'auto-source-badge runtime';
          sourceBadge.textContent = 'runtime';
          sourceBadge.title = 'Runtime-only config from NetworkTables';
        } else if (deploySet[name]) {
          sourceBadge.className = 'auto-source-badge deploy';
          sourceBadge.textContent = 'deploy';
          sourceBadge.title = 'Deployed config from robot code';
        } else if (localSet[name]) {
          sourceBadge.className = 'auto-source-badge local';
          sourceBadge.textContent = 'local';
          sourceBadge.title = 'Local editor config (not on robot yet)';
        }
        if (sourceBadge.textContent) {
          item.appendChild(sourceBadge);
        }

        var deleteBtn = document.createElement('button');
        deleteBtn.className = 'delete-auto-btn';
        deleteBtn.innerHTML = '&#x2715;';
        var isDeployOnly = !!deploySet[name] && !runtimeSet[name] && !localSet[name];
        deleteBtn.title = isDeployOnly ? 'Deploy configs are immutable' : 'Delete auto';
        if (isDeployOnly) {
          deleteBtn.disabled = true;
        }
        deleteBtn.addEventListener('click', function (e) {
          e.stopPropagation();
          if (confirm('Delete "' + name + '"? This will also delete it on the robot if connected.')) {
            deleteAutoConfig(name);
          }
        });
        item.appendChild(deleteBtn);

        item.addEventListener('click', function () {
          if (state.currentConfig && state.currentConfig.name === name) {
            return;
          }
          if (!confirmDiscardChanges('Load "' + name + '"')) {
            return;
          }

          // If connected, always fetch latest from robot.
          if (NT.isConnected()) {
            if (state.autoConfigSupported === false) {
              if (configCache[name]) {
                loadConfig(configCache[name]);
                showToast('Loaded cached "' + name + '" (robot does not support auto-config bridge)', 'info');
              } else {
                showToast('Connected robot does not support auto-config bridge topics', 'error');
              }
              return;
            }
            pendingLoadName = name;
            NT.requestConfig(name);
            if (configCache[name]) {
              loadConfig(configCache[name]);
              showToast('Loaded cached "' + name + '" (refreshing from robot...)', 'info');
            } else {
              showToast('Loading "' + name + '" from robot...', 'info');
            }
            return;
          }

          // Offline fallback: use cache if available.
          if (configCache[name]) {
            loadConfig(configCache[name]);
            showToast('Loaded cached "' + name + '"', 'success');
          } else {
            showToast('Not connected to robot. Use Import to load a JSON file.', 'error');
          }
        });

        dom.autoList.appendChild(item);
      })(allNames[i]);
    }
  }

  function createNewAuto() {
    var rawName = dom.newAutoName.value.trim();
    if (!rawName) { showToast('Enter a name for the new auto', 'error'); return; }
    if (!confirmDiscardChanges('Create a new auto')) {
      return;
    }
    var name = sanitizeAutoName(rawName);
    if (!name) { showToast('Auto name must include letters or numbers', 'error'); return; }

    // Track locally so it shows in the list
    if (localAutoNames.indexOf(name) === -1) {
      localAutoNames.push(name);
    }

    state.currentConfig = { name: name, description: '', version: 1, startingHeadingDegrees: 0 };
    state.steps = [];
    state.selectedStepId = null;
    state.selectedParentId = null;
    dom.newAutoName.value = '';
    syncConfigFields();
    history = [];
    historyIndex = -1;
    savedHistoryIndex = -1;
    pushHistory();
    setDirty(true);
    renderSequence();
    renderAutoList();
    renderProperties();
    updateActionButtons();
    if (name !== rawName) showToast('Name normalized to "' + name + '"', 'info');
    else showToast('Created new auto: ' + name, 'info');
  }

  function deleteAutoConfig(name) {
    if (state.currentConfig && state.currentConfig.name === name && !confirmDiscardChanges('Delete "' + name + '"')) {
      return;
    }
    if (NT.isConnected() && state.autoConfigSupported === true) NT.deleteConfig(name);
    state.autoNames = state.autoNames.filter(function (n) { return n !== name; });
    localAutoNames = localAutoNames.filter(function (n) { return n !== name; });
    delete configCache[name];
    if (state.currentConfig && state.currentConfig.name === name) {
      state.currentConfig = null;
      state.steps = [];
      state.selectedStepId = null;
      state.selectedParentId = null;
      history = [];
      historyIndex = -1;
      savedHistoryIndex = -1;
      setDirty(false);
      syncConfigFields();
      renderProperties();
    }
    renderAutoList();
    renderSequence();
    updateActionButtons();
  }

  // ===== Config Serialization =====

  function loadConfig(configJson) {
    var normalizedName = sanitizeAutoName(configJson.name || 'Untitled');
    if (!normalizedName) normalizedName = 'untitled_auto';
    state.currentConfig = {
      name: normalizedName,
      description: configJson.description || '',
      version: configJson.version || 1,
      startingHeadingDegrees: configJson.startingHeadingDegrees || 0,
    };

    state.steps = (configJson.steps || []).map(function (s) {
      var step = CP.deserializeStep(s);
      step._id = CP.generateStepId();
      if (step.type === 'parallel' && step.commands) {
        step.commands = step.commands.map(function (c) {
          var child = CP.deserializeStep(c);
          child._id = CP.generateStepId();
          return child;
        });
      }
      return step;
    });

    state.selectedStepId = null;
    state.selectedParentId = null;
    DD.clearMultiSelect();
    syncConfigFields();
    history = [];
    historyIndex = -1;
    savedHistoryIndex = -1;
    pushHistory();
    savedHistoryIndex = historyIndex;
    setDirty(false);
    renderSequence();
    renderAutoList();
    renderProperties();
    updateActionButtons();
  }

  function serializeConfig() {
    if (!state.currentConfig) return null;
    return {
      name: state.currentConfig.name,
      description: state.currentConfig.description,
      version: state.currentConfig.version,
      startingHeadingDegrees: state.currentConfig.startingHeadingDegrees,
      steps: state.steps.map(function (step) { return CP.serializeStep(step); }),
    };
  }

  function collectValidationErrors() {
    var allErrors = [];
    state.steps.forEach(function (step, i) {
      CP.validateStep(step).forEach(function (err) {
        allErrors.push('Step ' + (i + 1) + ' (' + step.type + '): ' + err);
      });
      if (step.type === 'parallel' && step.commands) {
        step.commands.forEach(function (child, ci) {
          CP.validateStep(child).forEach(function (err) {
            allErrors.push('Step ' + (i + 1) + ' > ' + (ci + 1) + ' (' + child.type + '): ' + err);
          });
        });
      }
    });
    return allErrors;
  }

  function estimateStepDuration(step) {
    if (!step) return { min: 0, max: 0, unbounded: false };
    if (step.type === 'drive') {
      if (step.mode === 'time') {
        var t = Math.max(0, step.durationSeconds || 0);
        return { min: t, max: t, unbounded: false };
      }
      if (step.mode === 'velocity') {
        var dv = Math.max(0, step.durationSeconds || 0);
        return { min: dv, max: dv, unbounded: false };
      }
      if (step.mode === 'to_pose') {
        return { min: 0, max: Math.max(0, step.timeoutSeconds || 0), unbounded: false };
      }
      var expected = 0;
      if (step.speedMPS > 0) {
        expected = Math.max(0, (step.distanceMetres || 0) / step.speedMPS);
      }
      var timeout = Math.max(0, step.timeoutSeconds || 0);
      return { min: expected, max: Math.max(expected, timeout), unbounded: false };
    }
    if (step.type === 'set_pose') {
      return { min: 0, max: 0, unbounded: false };
    }
    if (step.type === 'rotate') {
      return { min: 0, max: Math.max(0, step.timeoutSeconds || 0), unbounded: false };
    }
    if (step.type === 'face_target') {
      return { min: 0, max: Math.max(0, step.timeoutSeconds || 0), unbounded: false };
    }
    if (step.type === 'vision_approach_tag') {
      return { min: 0, max: Math.max(0, step.timeoutSeconds || 0), unbounded: false };
    }
    if (step.type === 'delay') {
      var d = Math.max(0, step.durationSeconds || 0);
      return { min: d, max: d, unbounded: false };
    }
    if (step.type === 'hold') {
      if (!step.durationSeconds || step.durationSeconds <= 0) {
        return { min: 0, max: Infinity, unbounded: true };
      }
      var h = Math.max(0, step.durationSeconds || 0);
      return { min: h, max: h, unbounded: false };
    }
    if (step.type === 'shooter') {
      if (step.action === 'on_with_duration') {
        var sd = Math.max(0, step.durationSeconds || 0);
        return { min: sd, max: sd, unbounded: false };
      }
      if (step.action === 'off') {
        return { min: 0, max: 0, unbounded: false };
      }
      return { min: 0, max: Infinity, unbounded: true };
    }
    if (step.type === 'intake') {
      if (step.action === 'on_with_duration') {
        var id = Math.max(0, step.durationSeconds || 0);
        return { min: id, max: id, unbounded: false };
      }
      if (step.action === 'off') {
        return { min: 0, max: 0, unbounded: false };
      }
      return { min: 0, max: Infinity, unbounded: true };
    }
    if (step.type === 'parallel') {
      var children = step.commands || [];
      if (!children.length) return { min: 0, max: 0, unbounded: false };
      var childDurations = children.map(estimateStepDuration);
      if (step.endCondition === 'first') {
        var minFirst = Infinity;
        var maxFirst = Infinity;
        var boundedCount = 0;
        childDurations.forEach(function (dInfo) {
          minFirst = Math.min(minFirst, dInfo.min);
          if (isFinite(dInfo.max)) {
            maxFirst = Math.min(maxFirst, dInfo.max);
            boundedCount++;
          }
        });
        return {
          min: isFinite(minFirst) ? minFirst : 0,
          max: boundedCount > 0 && isFinite(maxFirst) ? maxFirst : Infinity,
          unbounded: boundedCount === 0,
        };
      }
      if (step.endCondition === 'deadline') {
        var idx = Math.max(0, Math.min((step.deadlineIndex || 0), childDurations.length - 1));
        var deadlineDuration = childDurations[idx];
        return {
          min: deadlineDuration.min,
          max: deadlineDuration.max,
          unbounded: deadlineDuration.unbounded,
        };
      }
      var minAll = 0;
      var maxAll = 0;
      var anyUnbounded = false;
      childDurations.forEach(function (dInfo) {
        minAll = Math.max(minAll, dInfo.min);
        if (!isFinite(dInfo.max)) anyUnbounded = true;
        maxAll = Math.max(maxAll, isFinite(dInfo.max) ? dInfo.max : 0);
      });
      return { min: minAll, max: anyUnbounded ? Infinity : maxAll, unbounded: anyUnbounded };
    }
    return { min: 0, max: 0, unbounded: false };
  }

  function estimateSequenceDuration() {
    var totalMin = 0;
    var totalMax = 0;
    var unbounded = false;
    for (var i = 0; i < state.steps.length; i++) {
      var dInfo = estimateStepDuration(state.steps[i]);
      totalMin += dInfo.min;
      if (isFinite(dInfo.max)) {
        totalMax += dInfo.max;
      } else {
        unbounded = true;
      }
    }
    return { min: totalMin, max: unbounded ? Infinity : totalMax, unbounded: unbounded };
  }

  function formatSeconds(sec) {
    return (Math.round(sec * 10) / 10).toFixed(1) + 's';
  }

  function updateSequenceStats() {
    if (!dom.sequenceStats) return;
    if (!state.currentConfig) {
      dom.sequenceStats.textContent = 'No auto selected';
      return;
    }
    var validationErrors = collectValidationErrors();
    var dur = estimateSequenceDuration();
    var durationText = dur.unbounded
      ? ('>= ' + formatSeconds(dur.min))
      : (formatSeconds(dur.min) + ' - ' + formatSeconds(dur.max));

    var html = [];
    html.push('Steps: <strong>' + state.steps.length + '</strong>');
    html.push('Estimated runtime: <strong>' + durationText + '</strong>');
    if (dur.unbounded) {
      html.push('<span class="warn">contains run-until-interrupted step(s)</span>');
    }
    if (validationErrors.length > 0) {
      html.push('<span class="bad">' + validationErrors.length + ' validation issue(s)</span>');
    } else {
      html.push('<span class="status-message success" style="max-width:none;">valid</span>');
    }
    dom.sequenceStats.innerHTML = html.join('  •  ');
  }

  function updateActionButtons() {
    var hasSelection = !!state.selectedStepId;
    var hasSteps = state.steps.length > 0;
    var hasConfig = !!state.currentConfig;

    dom.undoBtn.disabled = historyIndex <= 0;
    dom.redoBtn.disabled = historyIndex >= history.length - 1 || historyIndex < 0;
    dom.moveUpBtn.disabled = !hasSelection;
    dom.moveDownBtn.disabled = !hasSelection;
    dom.duplicateSelectedBtn.disabled = !hasSelection;
    dom.deleteSelectedBtn.disabled = !hasSelection;
    dom.clearSequenceBtn.disabled = !hasConfig || !hasSteps;
    dom.validateBtn.disabled = !hasConfig;
    dom.saveBtn.disabled = !hasConfig || !state.connected || state.autoConfigSupported !== true;
    dom.saveRepoBtn.disabled = !hasConfig;
    dom.exportBtn.disabled = !hasConfig;
  }

  function validateCurrentConfig() {
    if (!state.currentConfig) {
      showToast('No auto loaded', 'error');
      return;
    }
    var errors = collectValidationErrors();
    if (errors.length === 0) {
      showToast('Validation passed', 'success');
      return;
    }
    showToast('Validation errors:\n' + errors.join('\n'), 'error');
  }

  // ===== Save / Export / Import =====

  function saveToRobot() {
    var config = serializeConfig();
    if (!config) { showToast('No auto config to save', 'error'); return; }
    if (!NT.isConnected()) { showToast('Not connected to robot. Use Export to save locally.', 'error'); return; }
    if (state.autoConfigSupported !== true) {
      showToast('Connected robot does not support this auto-config NetworkTables bridge', 'error');
      return;
    }

    var originalName = config.name;
    var safeName = sanitizeAutoName(config.name);
    if (!safeName) { showToast('Auto name must include letters or numbers', 'error'); return; }
    if (safeName !== config.name) {
      config.name = safeName;
      state.currentConfig.name = safeName;
      localAutoNames = localAutoNames.map(function (n) { return n === originalName ? safeName : n; });
      dom.configName.value = safeName;
      renderAutoList();
      showToast('Auto name normalized to "' + safeName + '" before save', 'info');
    }

    var allErrors = collectValidationErrors();
    if (allErrors.length > 0) { showToast('Validation errors:\n' + allErrors.join('\n'), 'error'); return; }

    pendingRobotSync = { name: config.name, hash: hashConfig(config) };
    NT.saveConfig(config);
    showToast('Saving "' + config.name + '" to robot...', 'info');
  }

  async function saveToRepoFile() {
    var config = serializeConfig();
    if (!config) { showToast('No auto config to save', 'error'); return; }

    var originalName = config.name;
    var safeName = sanitizeAutoName(config.name);
    if (!safeName) { showToast('Auto name must include letters or numbers', 'error'); return; }
    if (safeName !== config.name) {
      config.name = safeName;
      state.currentConfig.name = safeName;
      localAutoNames = localAutoNames.map(function (n) { return n === originalName ? safeName : n; });
      dom.configName.value = safeName;
      renderAutoList();
      showToast('Auto name normalized to "' + safeName + '" before repo save', 'info');
    }

    var allErrors = collectValidationErrors();
    if (allErrors.length > 0) { showToast('Validation errors:\n' + allErrors.join('\n'), 'error'); return; }

    var json = JSON.stringify(config, null, 2);
    var fileName = safeName + '.json';
    var savedToRepoDir = false;

    if (window.showDirectoryPicker) {
      try {
        if (!repoAutosDirHandle) {
          showToast('Select your repo autos folder (src/main/deploy/autos)', 'info');
          repoAutosDirHandle = await window.showDirectoryPicker({ mode: 'readwrite' });
        }
        var repoFileHandle = await repoAutosDirHandle.getFileHandle(fileName, { create: true });
        var repoWritable = await repoFileHandle.createWritable();
        await repoWritable.write(json);
        await repoWritable.close();
        savedToRepoDir = true;
      } catch (err) {
        if (err && err.name === 'AbortError') {
          return;
        }
        showToast('Could not write to selected repo folder; falling back to download', 'error');
      }
    }

    if (!savedToRepoDir) {
      var blob = new Blob([json], { type: 'application/json' });
      var url = URL.createObjectURL(blob);
      var a = document.createElement('a');
      a.href = url;
      a.download = fileName;
      a.click();
      URL.revokeObjectURL(url);
      showToast('Downloaded "' + fileName + '". Move it to src/main/deploy/autos/ (not repo-synced yet)', 'info');
    } else {
      showToast('Saved "' + fileName + '" to repo autos folder', 'success');
      repoSyncedHashesByName[safeName] = hashConfig(config);
    }

    pushHistory();
    savedHistoryIndex = historyIndex;
    setDirty(false);
    updateEditStatus();
    updateActionButtons();
  }

  function exportToFile() {
    var config = serializeConfig();
    if (!config) { showToast('No auto config to export', 'error'); return; }
    var json = JSON.stringify(config, null, 2);
    var blob = new Blob([json], { type: 'application/json' });
    var url = URL.createObjectURL(blob);
    var a = document.createElement('a');
    a.href = url;
    a.download = (config.name || 'auto-config').replace(/[^a-zA-Z0-9_-]/g, '_') + '.json';
    a.click();
    URL.revokeObjectURL(url);
    showToast('Exported "' + config.name + '"', 'success');
  }

  function importFromFile(e) {
    var file = e.target.files[0];
    if (!file) return;
    if (!confirmDiscardChanges('Import this file')) {
      e.target.value = '';
      return;
    }
    var reader = new FileReader();
    reader.onload = function (evt) {
      try {
        var config = JSON.parse(evt.target.result);
        if (!config.name || !Array.isArray(config.steps)) {
          showToast('Invalid auto config file: missing name or steps', 'error');
          return;
        }
        loadConfig(config);
        savedHistoryIndex = -1;
        setDirty(true);
        showToast('Imported "' + config.name + '"', 'success');
      } catch (err) {
        showToast('Failed to parse JSON: ' + err.message, 'error');
      }
    };
    reader.readAsText(file);
    e.target.value = '';
  }

  async function importLocalFolderFiles(e) {
    var files = Array.prototype.slice.call((e && e.target && e.target.files) || []);
    if (!files.length) return;

    var loaded = 0;
    for (var i = 0; i < files.length; i++) {
      var file = files[i];
      if (!file || !file.name || !file.name.toLowerCase().endsWith('.json')) continue;
      try {
        var text = await file.text();
        var config = JSON.parse(text);
        if (addLocalConfig(config)) {
          loaded++;
        }
      } catch (err) {
        // Ignore invalid files in folder.
      }
    }

    if (loaded > 0) {
      renderAutoList();
      updateEditStatus();
      showToast('Loaded ' + loaded + ' local auto config(s)', 'success');
    } else {
      showToast('No valid .json auto configs found in selected folder', 'error');
    }
    e.target.value = '';
  }

  // ===== Sequence Editor Rendering =====

  function renderSequence() {
    dom.sequenceEditor.innerHTML = '';

    if (!state.currentConfig) {
      dom.sequenceEditor.innerHTML =
        '<div class="sequence-empty"><div>No auto selected</div>' +
        '<div class="hint">Create a new auto or load one from the robot.</div></div>';
      updateWrapToolbar();
      updateSequenceStats();
      updateActionButtons();
      return;
    }

    if (state.steps.length === 0) {
      dom.sequenceEditor.innerHTML =
        '<div class="sequence-empty"><div>Empty sequence</div>' +
        '<div class="hint">Drag commands from the palette on the left, or click them to add.</div></div>';
      DD.setupSequenceDrag(dom.sequenceEditor);
      updateWrapToolbar();
      updateSequenceStats();
      updateActionButtons();
      return;
    }

    state.steps.forEach(function (step, index) {
      var isSelected = step._id === state.selectedStepId;
      var card = DD.createStepCard(step, index, isSelected);
      if (card) dom.sequenceEditor.appendChild(card);
    });

    DD.setupSequenceDrag(dom.sequenceEditor);
    updateWrapToolbar();
    updateSequenceStats();
    updateActionButtons();
  }

  // ===== Properties Panel =====

  function renderProperties() {
    if (!state.selectedStepId) {
      dom.propertiesContainer.innerHTML = '<div class="properties-empty">Select a command in the sequence to edit its properties.</div>';
      updateActionButtons();
      return;
    }
    var step = findStepById(state.selectedStepId);
    if (!step) {
      dom.propertiesContainer.innerHTML = '<div class="properties-empty">Step not found.</div>';
      updateActionButtons();
      return;
    }
    CP.renderPropertiesForm(dom.propertiesContainer, step,
      function (key, value) {
        step[key] = value;
        markDirty();
        renderSequence();
        renderProperties();
      },
      function () { deleteStep(state.selectedStepId); }
    );
    updateActionButtons();
  }

  // ===== Step Manipulation =====

  function findStepById(id) {
    for (var i = 0; i < state.steps.length; i++) {
      if (state.steps[i]._id === id) return state.steps[i];
      if (state.steps[i].type === 'parallel' && state.steps[i].commands) {
        for (var j = 0; j < state.steps[i].commands.length; j++) {
          if (state.steps[i].commands[j]._id === id) return state.steps[i].commands[j];
        }
      }
    }
    return null;
  }

  function findStepIndex(id) {
    for (var i = 0; i < state.steps.length; i++) {
      if (state.steps[i]._id === id) return i;
    }
    return -1;
  }

  function findChildLocation(id) {
    for (var i = 0; i < state.steps.length; i++) {
      var parent = state.steps[i];
      if (parent.type === 'parallel' && parent.commands) {
        for (var j = 0; j < parent.commands.length; j++) {
          if (parent.commands[j]._id === id) {
            return { parent: parent, parentIndex: i, childIndex: j };
          }
        }
      }
    }
    return null;
  }

  function moveSelectedStep(delta) {
    if (!state.selectedStepId) return;

    var topIndex = findStepIndex(state.selectedStepId);
    if (topIndex !== -1) {
      var targetTop = topIndex + delta;
      if (targetTop < 0 || targetTop >= state.steps.length) return;
      var moving = state.steps.splice(topIndex, 1)[0];
      state.steps.splice(targetTop, 0, moving);
      markDirty();
      renderSequence();
      renderProperties();
      return;
    }

    var childLoc = findChildLocation(state.selectedStepId);
    if (!childLoc || !childLoc.parent.commands) return;
    var targetChild = childLoc.childIndex + delta;
    if (targetChild < 0 || targetChild >= childLoc.parent.commands.length) return;
    var child = childLoc.parent.commands.splice(childLoc.childIndex, 1)[0];
    childLoc.parent.commands.splice(targetChild, 0, child);
    markDirty();
    renderSequence();
    renderProperties();
  }

  function clearSequence() {
    if (!state.currentConfig || state.steps.length === 0) return;
    if (!window.confirm('Clear all steps from this auto?')) return;
    state.steps = [];
    state.selectedStepId = null;
    state.selectedParentId = null;
    DD.clearMultiSelect();
    markDirty();
    renderSequence();
    renderProperties();
  }

  function deleteStep(id) {
    var changed = false;
    var index = findStepIndex(id);
    if (index !== -1) {
      state.steps.splice(index, 1);
      changed = true;
    } else {
      for (var i = 0; i < state.steps.length; i++) {
        if (state.steps[i].type === 'parallel' && state.steps[i].commands) {
          var ci = state.steps[i].commands.findIndex(function (c) { return c._id === id; });
          if (ci !== -1) { state.steps[i].commands.splice(ci, 1); changed = true; break; }
        }
      }
    }
    if (!changed) return;
    if (state.selectedStepId === id) { state.selectedStepId = null; state.selectedParentId = null; }
    DD.clearMultiSelect();
    markDirty();
    renderSequence();
    renderProperties();
  }

  function duplicateStep(id) {
    var index = findStepIndex(id);
    if (index !== -1) {
      var copy = JSON.parse(JSON.stringify(state.steps[index]));
      copy._id = CP.generateStepId();
      if (copy.type === 'parallel' && copy.commands) {
        copy.commands = copy.commands.map(function (c) { c._id = CP.generateStepId(); return c; });
      }
      state.steps.splice(index + 1, 0, copy);
      state.selectedStepId = copy._id;
      state.selectedParentId = null;
      markDirty();
      renderSequence();
      renderProperties();
      return;
    }

    var childLoc = findChildLocation(id);
    if (!childLoc || !childLoc.parent || !childLoc.parent.commands) return;
    var childCopy = JSON.parse(JSON.stringify(childLoc.parent.commands[childLoc.childIndex]));
    childCopy._id = CP.generateStepId();
    childLoc.parent.commands.splice(childLoc.childIndex + 1, 0, childCopy);
    state.selectedStepId = childCopy._id;
    state.selectedParentId = childLoc.parent._id;
    markDirty();
    renderSequence();
    renderProperties();
  }

  function wrapInParallelGroup(stepIds) {
    if (!stepIds || stepIds.size < 2) return;
    var idsArr = Array.from(stepIds);
    var indices = idsArr
      .map(function (id) { return findStepIndex(id); })
      .filter(function (i) { return i !== -1; })
      .sort(function (a, b) { return a - b; });

    if (indices.length < 2) return;
    var stepsToWrap = indices.map(function (i) { return state.steps[i]; });
    if (stepsToWrap.some(function (s) { return s.type === 'parallel'; })) {
      showToast('Cannot nest parallel groups', 'error');
      return;
    }

    var parallelStep = Object.assign({}, CP.COMMAND_TYPES.parallel.defaultValues, {
      _id: CP.generateStepId(),
      commands: stepsToWrap,
    });

    var reversed = indices.slice().reverse();
    for (var i = 0; i < reversed.length; i++) state.steps.splice(reversed[i], 1);
    state.steps.splice(indices[0], 0, parallelStep);
    state.selectedStepId = parallelStep._id;
    state.selectedParentId = null;
    markDirty();
    renderSequence();
    renderProperties();
  }

  // ===== App Interface for Drag-Drop =====

  var appInterface = {
    getSteps: function () { return state.steps; },
    getSelectedStepId: function () { return state.selectedStepId; },
    insertStep: function (index, stepData) {
      state.steps.splice(index, 0, stepData);
      state.selectedStepId = stepData._id;
      state.selectedParentId = null;
      markDirty();
      renderSequence();
      renderProperties();
    },
    moveStep: function (fromIndex, toIndex) {
      var step = state.steps.splice(fromIndex, 1)[0];
      state.steps.splice(toIndex, 0, step);
      markDirty();
      renderSequence();
    },
    selectStep: function (id, parentId) {
      state.selectedStepId = id;
      state.selectedParentId = parentId || null;
      DD.clearMultiSelect();
      renderSequence();
      renderProperties();
      updateWrapToolbar();
    },
    toggleStepMultiSelect: function (id) {
      DD.toggleMultiSelect(id);
      updateWrapToolbar();
      renderSequence();
    },
    clearMultiSelect: function () {
      DD.clearMultiSelect();
      updateWrapToolbar();
      renderSequence();
    },
    isStepMultiSelected: function (id) {
      return DD.getMultiSelectIds().has(id);
    },
    deleteStep: function (id) { deleteStep(id); },
    duplicateStep: function (id) { duplicateStep(id); },
    deleteChildStep: function (parentId, childId) {
      var parent = findStepById(parentId);
      if (parent && parent.commands) {
        parent.commands = parent.commands.filter(function (c) { return c._id !== childId; });
        if (state.selectedStepId === childId) { state.selectedStepId = parentId; state.selectedParentId = null; }
        markDirty();
        renderSequence();
        renderProperties();
        updateWrapToolbar();
      }
    },
    addChildToParallel: function (parentId, childStep) {
      var parent = findStepById(parentId);
      if (parent && parent.type === 'parallel') {
        if (!parent.commands) parent.commands = [];
        parent.commands.push(childStep);
        markDirty();
        renderSequence();
        updateWrapToolbar();
      }
    },
    moveStepIntoParallel: function (stepId, parentId) {
      var stepIndex = findStepIndex(stepId);
      if (stepIndex === -1) return;
      var step = state.steps[stepIndex];
      if (step.type === 'parallel') return;
      var parent = findStepById(parentId);
      if (!parent || parent.type !== 'parallel') return;
      state.steps.splice(stepIndex, 1);
      if (!parent.commands) parent.commands = [];
      parent.commands.push(step);
      markDirty();
      renderSequence();
      updateWrapToolbar();
    },
    wrapInParallel: function (stepIds) { wrapInParallelGroup(stepIds); },
    renderSequence: function () { renderSequence(); },
  };

  // ===== Wrap Toolbar =====

  function updateWrapToolbar() {
    var multiIds = DD.getMultiSelectIds();
    if (multiIds.size >= 2) {
      dom.wrapToolbar.style.display = 'flex';
      dom.selectionInfo.textContent = multiIds.size + ' steps selected';
    } else {
      dom.wrapToolbar.style.display = 'none';
    }
  }

  // ===== Toast Notifications =====

  function showToast(message, type) {
    document.querySelectorAll('.toast').forEach(function (t) { t.remove(); });
    var toast = document.createElement('div');
    toast.className = 'toast ' + (type || 'info');
    toast.textContent = message;
    document.body.appendChild(toast);
    setTimeout(function () { toast.remove(); }, 3000);
  }
})();
