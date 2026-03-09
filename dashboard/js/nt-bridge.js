/**
 * NetworkTables 4 bridge for the FRC 1310 Auto Configuration Dashboard.
 * Uses the NTCore global from ntcore-bundle.js.
 */

window.NTBridge = (function () {
  'use strict';

  var ntClient = null;
  var ntConnected = false;

  var configSubscriptions = new Map();

  var onConnectionChange = function () {};
  var onAutoListUpdate = function () {};
  var onDeployAutoListUpdate = function () {};
  var onRuntimeAutoListUpdate = function () {};
  var onConfigReceived = function () {};
  var onWriteStatus = function () {};
  var onFeatureSupportChange = function () {};
  var onCommandMetadata = function () {};

  var NT_PREFIX = '/SmartDashboard/1310/autoconfig/';
  var FEATURE_DETECT_TIMEOUT_MS = 2500;
  var featureSupportState = null; // null=unknown, true=supported, false=unsupported
  var featureDetectTimer = null;

  function init(callbacks) {
    onConnectionChange = callbacks.onConnectionChange || function () {};
    onAutoListUpdate = callbacks.onAutoListUpdate || function () {};
    onDeployAutoListUpdate = callbacks.onDeployAutoListUpdate || function () {};
    onRuntimeAutoListUpdate = callbacks.onRuntimeAutoListUpdate || function () {};
    onConfigReceived = callbacks.onConfigReceived || function () {};
    onWriteStatus = callbacks.onWriteStatus || function () {};
    onFeatureSupportChange = callbacks.onFeatureSupportChange || function () {};
    onCommandMetadata = callbacks.onCommandMetadata || function () {};
  }

  function clearFeatureDetectTimer() {
    if (featureDetectTimer) {
      clearTimeout(featureDetectTimer);
      featureDetectTimer = null;
    }
  }

  function setFeatureSupport(state, message) {
    if (featureSupportState === state) return;
    featureSupportState = state;
    onFeatureSupportChange(state, message || '');
  }

  function markFeatureSupported() {
    clearFeatureDetectTimer();
    setFeatureSupport(true, 'Auto-config bridge detected');
  }

  function startFeatureSupportDetection() {
    clearFeatureDetectTimer();
    setFeatureSupport(null, 'Connected (checking auto-config support...)');
    featureDetectTimer = setTimeout(function () {
      if (ntConnected && featureSupportState !== true) {
        setFeatureSupport(false, 'Connected, but robot does not support 1310 auto-config topics');
      }
    }, FEATURE_DETECT_TIMEOUT_MS);
  }

  function connect(robotAddress) {
    if (!window.NTCore) {
      console.error('NTCore not loaded.');
      onConnectionChange(false, 'NT library not loaded.');
      return;
    }

    disconnect();

    try {
      var NetworkTables = window.NTCore.NetworkTables;
      ntClient = NetworkTables.getInstanceByURI(robotAddress);

      ntClient.addRobotConnectionListener(function (connected) {
        ntConnected = connected;
        onConnectionChange(connected, connected ? 'Connected' : 'Disconnected');
        if (connected) {
          startFeatureSupportDetection();
          subscribeToBridgeVersion();
          subscribeToAvailableAutos();
          subscribeToDeployAutos();
          subscribeToRuntimeAutos();
          subscribeToWriteStatus();
          subscribeToCommandMetadata();
        } else {
          clearFeatureDetectTimer();
          setFeatureSupport(null, '');
        }
      }, true);
    } catch (e) {
      console.error('NT connection error:', e);
      onConnectionChange(false, 'Connection error: ' + e.message);
    }
  }

  function disconnect() {
    configSubscriptions.clear();

    if (ntClient) {
      try { ntClient.close(); } catch (e) { /* ignore */ }
      ntClient = null;
    }

    clearFeatureDetectTimer();
    ntConnected = false;
    setFeatureSupport(null, '');
    onConnectionChange(false, 'Disconnected');
  }

  function requestConfig(name) {
    if (!ntClient || !ntConnected) return;

    var topicPath = NT_PREFIX + 'configs/' + name;

    if (configSubscriptions.has(name)) {
      configSubscriptions.delete(name);
    }

    try {
      var topic = ntClient.createTopic(topicPath, 'string');
      ntClient.subscribe(topic, function (value) {
        if (value !== null && value !== undefined) {
          try {
            var config = JSON.parse(value);
            if (config && config.steps && Array.isArray(config.steps)) {
              markFeatureSupported();
            }
            onConfigReceived(name, config);
          } catch (e) {
            console.error('Failed to parse config JSON for', name, ':', e);
          }
        }
      });
      configSubscriptions.set(name, topic);
    } catch (e) {
      console.error('Error subscribing to config:', e);
    }
  }

  function saveConfig(configJson) {
    if (!ntClient || !ntConnected) {
      onWriteStatus('Not connected to robot');
      return false;
    }

    try {
      var topic = ntClient.createTopic(NT_PREFIX + 'writeConfig', 'string');
      ntClient.setValue(topic, JSON.stringify(configJson));
      return true;
    } catch (e) {
      console.error('Error saving config:', e);
      onWriteStatus('Error: ' + e.message);
      return false;
    }
  }

  function deleteConfig(name) {
    if (!ntClient || !ntConnected) {
      onWriteStatus('Not connected to robot');
      return false;
    }

    try {
      var topic = ntClient.createTopic(NT_PREFIX + 'deleteConfig', 'string');
      ntClient.setValue(topic, name);
      return true;
    } catch (e) {
      console.error('Error deleting config:', e);
      onWriteStatus('Error: ' + e.message);
      return false;
    }
  }

  function isConnected() {
    return ntConnected;
  }

  function subscribeToAvailableAutos() {
    if (!ntClient) return;
    try {
      var topic = ntClient.createTopic(NT_PREFIX + 'availableAutos', 'string[]');
      ntClient.subscribe(topic, function (value) {
        if (Array.isArray(value)) {
          if (value.length > 0) {
            markFeatureSupported();
          }
          onAutoListUpdate(value);
        }
      });
    } catch (e) {
      console.error('Error subscribing to availableAutos:', e);
    }
  }

  function subscribeToRuntimeAutos() {
    if (!ntClient) return;
    try {
      var topic = ntClient.createTopic(NT_PREFIX + 'runtimeAutos', 'string[]');
      ntClient.subscribe(topic, function (value) {
        if (Array.isArray(value)) {
          if (value.length > 0) {
            markFeatureSupported();
          }
          onRuntimeAutoListUpdate(value);
        }
      });
    } catch (e) {
      console.error('Error subscribing to runtimeAutos:', e);
    }
  }

  function subscribeToDeployAutos() {
    if (!ntClient) return;
    try {
      var topic = ntClient.createTopic(NT_PREFIX + 'deployAutos', 'string[]');
      ntClient.subscribe(topic, function (value) {
        if (Array.isArray(value)) {
          if (value.length > 0) {
            markFeatureSupported();
          }
          onDeployAutoListUpdate(value);
        }
      });
    } catch (e) {
      console.error('Error subscribing to deployAutos:', e);
    }
  }

  function subscribeToWriteStatus() {
    if (!ntClient) return;
    try {
      var topic = ntClient.createTopic(NT_PREFIX + 'lastWriteStatus', 'string');
      ntClient.subscribe(topic, function (value) {
        if (value !== null && value !== undefined) {
          if (String(value)) {
            markFeatureSupported();
          }
          onWriteStatus(String(value));
        }
      });
    } catch (e) {
      console.error('Error subscribing to lastWriteStatus:', e);
    }
  }

  function subscribeToCommandMetadata() {
    if (!ntClient) return;
    try {
      var topic = ntClient.createTopic(NT_PREFIX + 'commandMetadata', 'string');
      ntClient.subscribe(topic, function (value) {
        if (typeof value === 'string' && value.length > 0) {
          markFeatureSupported();
          try {
            var metadata = JSON.parse(value);
            onCommandMetadata(metadata);
          } catch (e) {
            console.error('Failed to parse command metadata:', e);
          }
        }
      });
    } catch (e) {
      console.error('Error subscribing to commandMetadata:', e);
    }
  }

  function subscribeToBridgeVersion() {
    if (!ntClient) return;
    try {
      var topic = ntClient.createTopic(NT_PREFIX + 'bridgeVersion', 'string');
      ntClient.subscribe(topic, function (value) {
        if (typeof value === 'string' && value.trim().length > 0) {
          markFeatureSupported();
        }
      });
    } catch (e) {
      console.error('Error subscribing to bridgeVersion:', e);
    }
  }

  return {
    init: init,
    connect: connect,
    disconnect: disconnect,
    requestConfig: requestConfig,
    saveConfig: saveConfig,
    deleteConfig: deleteConfig,
    isConnected: isConnected,
  };
})();
