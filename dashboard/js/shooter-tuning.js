/**
 * Shooter Tuning Dashboard logic for FRC 1310.
 * Uses NTCore global from ntcore-bundle.js.
 */
(function () {
  'use strict';

  var NT_PREFIX = '/SmartDashboard/1310/shootertune/';
  var ntClient = null;
  var ntConnected = false;
  var heartbeatCounter = 0;
  var heartbeatInterval = null;

  // Topic handles
  var topics = {};

  // DOM refs
  var statusDot, statusText, connectBtn, teamInput;
  var rpmDisplay, atSpeedDot, atSpeedText, distanceDisplay, angleDisplay;
  var rpmInput, rpmSlider, shooterToggle;
  var hoodSlider, hoodValue;
  var kickerSlider, kickerValue, kickerToggle, autoKickerToggle;
  var recordBtn, exportCsvBtn, clearRecordingsBtn, recordingsBody, recordCount, measuredDistance;
  var autoKickerEnabled = false;

  var STORAGE_KEY = '1310-shooter-recordings';
  var SETTINGS_KEY = '1310-shooter-settings';

  function init() {
    statusDot = document.getElementById('status-dot');
    statusText = document.getElementById('status-text');
    connectBtn = document.getElementById('connect-btn');
    teamInput = document.getElementById('team-number');
    rpmDisplay = document.getElementById('rpm-display');
    atSpeedDot = document.getElementById('at-speed-dot');
    atSpeedText = document.getElementById('at-speed-text');
    distanceDisplay = document.getElementById('distance-display');
    angleDisplay = document.getElementById('angle-display');
    rpmInput = document.getElementById('rpm-input');
    rpmSlider = document.getElementById('rpm-slider');
    shooterToggle = document.getElementById('shooter-toggle');
    hoodSlider = document.getElementById('hood-slider');
    hoodValue = document.getElementById('hood-value');
    kickerSlider = document.getElementById('kicker-slider');
    kickerValue = document.getElementById('kicker-value');
    kickerToggle = document.getElementById('kicker-toggle');
    autoKickerToggle = document.getElementById('auto-kicker-toggle');

    connectBtn.addEventListener('click', handleConnect);

    // Restore saved settings
    loadSettings();

    rpmInput.addEventListener('input', function () {
      rpmSlider.value = rpmInput.value;
      publishDouble('targetRPM', parseFloat(rpmInput.value) || 0);
      saveSettings();
    });
    rpmSlider.addEventListener('input', function () {
      rpmInput.value = rpmSlider.value;
      publishDouble('targetRPM', parseFloat(rpmSlider.value) || 0);
      saveSettings();
    });

    shooterToggle.addEventListener('click', function () {
      var active = shooterToggle.classList.toggle('active');
      shooterToggle.textContent = active ? 'ON' : 'OFF';
      publishBoolean('shooterEnabled', active);
      // When turning shooter off, disable kicker but leave auto-kicker mode intact
      if (!active) {
        kickerToggle.classList.remove('active');
        kickerToggle.textContent = 'OFF';
        publishBoolean('kickerEnabled', false);
      }
    });

    hoodSlider.addEventListener('input', function () {
      hoodValue.textContent = parseFloat(hoodSlider.value).toFixed(2);
      publishDouble('hoodAngle', parseFloat(hoodSlider.value) || 0);
      saveSettings();
    });

    kickerSlider.addEventListener('input', function () {
      var negated = -(parseFloat(kickerSlider.value) || 0);
      kickerValue.textContent = negated.toFixed(2);
      publishDouble('kickerSpeed', negated);
      saveSettings();
    });

    kickerToggle.addEventListener('click', function () {
      var active = kickerToggle.classList.toggle('active');
      kickerToggle.textContent = active ? 'ON' : 'OFF';
      publishBoolean('kickerEnabled', active);
      // Turn off auto-kicker when manually toggling
      if (autoKickerEnabled) {
        autoKickerEnabled = false;
        autoKickerToggle.classList.remove('active');
        autoKickerToggle.textContent = 'AUTO';
      }
    });

    measuredDistance = document.getElementById('measured-distance');
    recordBtn = document.getElementById('record-btn');
    exportCsvBtn = document.getElementById('export-csv-btn');
    clearRecordingsBtn = document.getElementById('clear-recordings-btn');
    recordingsBody = document.getElementById('recordings-body');
    recordCount = document.getElementById('record-count');

    recordBtn.addEventListener('click', recordSettings);
    exportCsvBtn.addEventListener('click', exportCSV);
    clearRecordingsBtn.addEventListener('click', function () {
      if (confirm('Clear all recordings?')) {
        localStorage.removeItem(STORAGE_KEY);
        renderRecordings();
      }
    });

    renderRecordings();

    autoKickerToggle.addEventListener('click', function () {
      autoKickerEnabled = !autoKickerEnabled;
      autoKickerToggle.classList.toggle('active', autoKickerEnabled);
      autoKickerToggle.textContent = autoKickerEnabled ? 'AUTO ON' : 'AUTO';
      // If turning off auto-kicker, also disable kicker
      if (!autoKickerEnabled) {
        kickerToggle.classList.remove('active');
        kickerToggle.textContent = 'OFF';
        publishBoolean('kickerEnabled', false);
      }
    });
  }

  function teamToIP(team) {
    var num = parseInt(team, 10);
    if (isNaN(num) || num < 0 || num > 9999) return null;
    var te = Math.floor(num / 100);
    var am = num % 100;
    return '10.' + te + '.' + am + '.2';
  }

  function handleConnect() {
    if (ntClient) {
      disconnect();
      return;
    }
    var team = teamInput.value.trim();
    var ip = teamToIP(team);
    if (!ip) {
      statusText.textContent = 'Invalid team number';
      return;
    }
    connect(ip);
  }

  function connect(robotAddress) {
    if (!window.NTCore) {
      statusText.textContent = 'NT library not loaded';
      return;
    }
    disconnect();

    try {
      var NetworkTables = window.NTCore.NetworkTables;
      ntClient = NetworkTables.getInstanceByURI(robotAddress);
      ntClient.addRobotConnectionListener(function (connected) {
        ntConnected = connected;
        updateConnectionUI(connected);
        if (connected) {
          // Clear stale topic handles so they get recreated
          topics = {};
          subscribeToFeedback();
          publishAllCurrentValues();
          startHeartbeat();
        }
      }, true);
    } catch (e) {
      statusText.textContent = 'Error: ' + e.message;
    }
  }

  function startHeartbeat() {
    stopHeartbeat();
    heartbeatInterval = setInterval(function () {
      heartbeatCounter++;
      publishDouble('heartbeat', heartbeatCounter);
    }, 500);
  }

  function stopHeartbeat() {
    if (heartbeatInterval) {
      clearInterval(heartbeatInterval);
      heartbeatInterval = null;
    }
  }

  function disconnect() {
    stopHeartbeat();
    if (ntClient) {
      try { ntClient.close(); } catch (e) { /* ignore */ }
      ntClient = null;
    }
    ntConnected = false;
    topics = {};
    updateConnectionUI(false);
  }

  function updateConnectionUI(connected) {
    statusDot.className = 'status-dot ' + (connected ? 'connected' : '');
    if (connected) {
      statusText.textContent = 'Connected';
    } else if (ntClient) {
      statusText.textContent = 'Reconnecting...';
    } else {
      statusText.textContent = 'Disconnected';
    }
    connectBtn.textContent = ntClient ? 'Disconnect' : 'Connect';
  }

  function getTopic(name, type) {
    if (!topics[name]) {
      topics[name] = ntClient.createTopic(NT_PREFIX + name, type);
    }
    return topics[name];
  }

  function publishDouble(name, value) {
    if (!ntClient || !ntConnected) return;
    try {
      ntClient.setValue(getTopic(name, 'double'), value);
    } catch (e) {
      console.error('Error publishing ' + name + ':', e);
    }
  }

  function publishBoolean(name, value) {
    if (!ntClient || !ntConnected) return;
    try {
      ntClient.setValue(getTopic(name, 'boolean'), value);
    } catch (e) {
      console.error('Error publishing ' + name + ':', e);
    }
  }

  function publishAllCurrentValues() {
    publishDouble('targetRPM', parseFloat(rpmInput.value) || 0);
    publishDouble('hoodAngle', parseFloat(hoodSlider.value) || 0);
    publishDouble('kickerSpeed', -(parseFloat(kickerSlider.value) || 0));
    publishBoolean('shooterEnabled', shooterToggle.classList.contains('active'));
    publishBoolean('kickerEnabled', kickerToggle.classList.contains('active'));
  }

  function subscribeToFeedback() {
    if (!ntClient) return;
    try {
      var rpmTopic = getTopic('currentRPM', 'double');
      ntClient.subscribe(rpmTopic, function (value) {
        if (value !== null && value !== undefined) {
          rpmDisplay.textContent = Math.round(value);
        }
      });

      var atSpeedTopic = getTopic('atSpeed', 'boolean');
      ntClient.subscribe(atSpeedTopic, function (value) {
        var at = !!value;
        atSpeedDot.className = 'at-speed-dot ' + (at ? 'at-speed' : '');
        atSpeedText.textContent = at ? 'At Speed' : 'Not At Speed';

        // Auto-kicker: enable kicker once at speed, keep running until shooter is turned off
        if (autoKickerEnabled && at && !kickerToggle.classList.contains('active')) {
          kickerToggle.classList.add('active');
          kickerToggle.textContent = 'ON';
          publishBoolean('kickerEnabled', true);
        }
      });

      var distTopic = getTopic('distanceToHub', 'double');
      ntClient.subscribe(distTopic, function (value) {
        if (value !== null && value !== undefined) {
          distanceDisplay.textContent = value.toFixed(2);
        }
      });

      var angleTopic = getTopic('angleToHub', 'double');
      ntClient.subscribe(angleTopic, function (value) {
        if (value !== null && value !== undefined) {
          angleDisplay.textContent = value.toFixed(1);
        }
      });
    } catch (e) {
      console.error('Error subscribing to feedback:', e);
    }
  }

  function getRecordings() {
    try {
      return JSON.parse(localStorage.getItem(STORAGE_KEY)) || [];
    } catch (e) {
      return [];
    }
  }

  function saveRecordings(recordings) {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(recordings));
  }

  function recordSettings() {
    var recordings = getRecordings();
    recordings.push({
      distance: distanceDisplay.textContent,
      measuredDistance: measuredDistance.value || '',
      angle: angleDisplay.textContent,
      targetRPM: rpmInput.value,
      hoodAngle: hoodSlider.value,
      kickerSpeed: kickerValue.textContent,
      note: ''
    });
    saveRecordings(recordings);
    renderRecordings();
  }

  function renderRecordings() {
    var recordings = getRecordings();
    recordingsBody.innerHTML = '';
    recordCount.textContent = recordings.length ? recordings.length + ' recorded' : '';

    for (var i = 0; i < recordings.length; i++) {
      (function (idx) {
        var r = recordings[idx];
        var tr = document.createElement('tr');
        tr.innerHTML =
          '<td>' + (idx + 1) + '</td>' +
          '<td>' + r.distance + '</td>' +
          '<td>' + (r.measuredDistance || '') + '</td>' +
          '<td>' + r.angle + '</td>' +
          '<td>' + r.targetRPM + '</td>' +
          '<td>' + r.hoodAngle + '</td>' +
          '<td>' + r.kickerSpeed + '</td>' +
          '<td><input class="note-input" type="text" value="' + (r.note || '').replace(/"/g, '&quot;') + '" placeholder="Add note..."></td>' +
          '<td><button class="delete-row-btn" title="Delete">&times;</button></td>';

        tr.querySelector('.note-input').addEventListener('change', function () {
          var recs = getRecordings();
          recs[idx].note = this.value;
          saveRecordings(recs);
        });

        tr.querySelector('.delete-row-btn').addEventListener('click', function () {
          var recs = getRecordings();
          recs.splice(idx, 1);
          saveRecordings(recs);
          renderRecordings();
        });

        recordingsBody.appendChild(tr);
      })(i);
    }
  }

  function exportCSV() {
    var recordings = getRecordings();
    if (!recordings.length) return;

    var lines = ['#,Odom Distance (m),Measured Distance (m),Angle (deg),Target RPM,Hood Angle,Kicker Speed,Note'];
    for (var i = 0; i < recordings.length; i++) {
      var r = recordings[i];
      var note = '"' + (r.note || '').replace(/"/g, '""') + '"';
      lines.push([i + 1, r.distance, r.measuredDistance || '', r.angle, r.targetRPM, r.hoodAngle, r.kickerSpeed, note].join(','));
    }

    var blob = new Blob([lines.join('\n')], { type: 'text/csv' });
    var a = document.createElement('a');
    a.href = URL.createObjectURL(blob);
    a.download = 'shooter-tuning-' + new Date().toISOString().slice(0, 10) + '.csv';
    a.click();
    URL.revokeObjectURL(a.href);
  }

  function saveSettings() {
    var settings = {
      targetRPM: rpmInput.value,
      hoodAngle: hoodSlider.value,
      kickerSpeed: kickerSlider.value
    };
    localStorage.setItem(SETTINGS_KEY, JSON.stringify(settings));
  }

  function loadSettings() {
    var s;
    try {
      s = JSON.parse(localStorage.getItem(SETTINGS_KEY));
    } catch (e) {
      s = null;
    }

    if (s) {
      rpmInput.value = s.targetRPM || 0;
      rpmSlider.value = s.targetRPM || 0;
      hoodSlider.value = s.hoodAngle || 0;
      hoodValue.textContent = parseFloat(s.hoodAngle || 0).toFixed(2);
      kickerSlider.value = s.kickerSpeed || 1;
      kickerValue.textContent = (-(parseFloat(s.kickerSpeed || 1))).toFixed(2);
    }

    // Default auto-kicker to ON
    autoKickerEnabled = true;
    autoKickerToggle.classList.add('active');
    autoKickerToggle.textContent = 'AUTO ON';
  }

  document.addEventListener('DOMContentLoaded', init);
})();
