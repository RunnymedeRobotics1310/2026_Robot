/**
 * ntcore-ts-client wrapper / stub module.
 *
 * Minimal NetworkTables 4 implementation using raw WebSocket.
 * Works without npm/esbuild — just open the HTML in a browser.
 *
 * For full reliability at competition, replace with the real ntcore-ts-client
 * bundle via `npm install && npm run build-ntcore`.
 */

window.NTCore = (function () {
  'use strict';

  const NT4_WS_PORT = 5810;

  class NT4Topic {
    constructor(name, type) {
      this.name = name;
      this.type = type;
      this.id = -1;
      this.listeners = [];
      this.pubuid = null;
      this.lastValue = undefined;
    }
  }

  class NT4Client {
    constructor(uri) {
      this.serverUri = uri;
      this.ws = null;
      this.connected = false;
      this.connectionListeners = [];
      this.topics = new Map();
      this.topicIdMap = new Map();
      this.nextSubUid = 1;
      this.nextPubUid = 1;
      this.reconnectTimer = null;

      this._connect();
    }

    _connect() {
      const wsUrl = 'ws://' + this.serverUri + ':' + NT4_WS_PORT + '/nt/frc1310-dashboard';

      try {
        this.ws = new WebSocket(wsUrl, ['networktables.first.wpi.edu']);
      } catch (e) {
        console.error('WebSocket creation failed:', e);
        this._scheduleReconnect();
        return;
      }

      this.ws.binaryType = 'arraybuffer';

      this.ws.onopen = () => {
        this.connected = true;
        this._notifyConnection(true);

        this._sendJson([{
          method: 'subscribe',
          params: {
            topics: ['/SmartDashboard/1310/autoconfig/'],
            subuid: this.nextSubUid++,
            options: { prefix: true, periodic: 0.1 },
          },
        }]);

        for (var entry of this.topics) {
          this._subscribeWs(entry[1]);
        }
      };

      this.ws.onmessage = (event) => {
        if (typeof event.data === 'string') {
          this._handleJsonMessage(event.data);
        } else {
          this._handleBinaryMessage(event.data);
        }
      };

      this.ws.onclose = () => {
        var wasConnected = this.connected;
        this.connected = false;
        if (wasConnected) {
          this._notifyConnection(false);
        }
        this._scheduleReconnect();
      };

      this.ws.onerror = function (e) {
        console.warn('NT4 WebSocket error:', e);
      };
    }

    _scheduleReconnect() {
      if (this.reconnectTimer) return;
      var self = this;
      this.reconnectTimer = setTimeout(function () {
        self.reconnectTimer = null;
        if (!self.connected) {
          self._connect();
        }
      }, 2000);
    }

    _sendJson(messages) {
      if (this.ws && this.ws.readyState === WebSocket.OPEN) {
        this.ws.send(JSON.stringify(messages));
      }
    }

    _handleJsonMessage(data) {
      try {
        var messages = JSON.parse(data);
        if (!Array.isArray(messages)) return;

        for (var i = 0; i < messages.length; i++) {
          var msg = messages[i];
          if (msg.method === 'announce') {
            var name = msg.params.name;
            var id = msg.params.id;
            var type = msg.params.type;
            var topic = this.topics.get(name);
            if (!topic) {
              topic = new NT4Topic(name, type);
              this.topics.set(name, topic);
            }
            topic.id = id;
            topic.type = type;
            this.topicIdMap.set(id, topic);
          } else if (msg.method === 'unannounce') {
            var uid = msg.params.id;
            this.topicIdMap.delete(uid);
          }
        }
      } catch (e) {
        console.warn('Failed to parse NT4 JSON message:', e);
      }
    }

    _handleBinaryMessage(data) {
      try {
        var decoded = decodeMsgpackMessages(new Uint8Array(data));
        for (var i = 0; i < decoded.length; i++) {
          var entry = decoded[i];
          if (entry.length >= 4) {
            var topicId = entry[0];
            var value = entry[3];
            var topic = this.topicIdMap.get(topicId);
            if (topic) {
              // Cache latest value so new listeners can get it immediately
              topic.lastValue = value;
              for (var j = 0; j < topic.listeners.length; j++) {
                topic.listeners[j](value);
              }
            }
          }
        }
      } catch (e) {
        // Silently ignore binary parse errors
      }
    }

    _subscribeWs(topic) {
      this._sendJson([{
        method: 'subscribe',
        params: {
          topics: [topic.name],
          subuid: this.nextSubUid++,
          options: { periodic: 0.1 },
        },
      }]);
    }

    _publishWs(topic) {
      this._sendJson([{
        method: 'publish',
        params: {
          name: topic.name,
          pubuid: this.nextPubUid++,
          type: topic.type,
          properties: {},
        },
      }]);
      topic.pubuid = this.nextPubUid - 1;
    }

    addRobotConnectionListener(callback, immediate) {
      this.connectionListeners.push(callback);
      if (immediate) {
        callback(this.connected);
      }
    }

    _notifyConnection(connected) {
      for (var i = 0; i < this.connectionListeners.length; i++) {
        this.connectionListeners[i](connected);
      }
    }

    createTopic(name, type) {
      var topic = this.topics.get(name);
      if (!topic) {
        topic = new NT4Topic(name, type);
        this.topics.set(name, topic);
      }
      return topic;
    }

    subscribe(topic, callback) {
      topic.listeners.push(callback);
      // If we already have a cached value, replay it immediately to the new listener
      if (topic.lastValue !== undefined) {
        callback(topic.lastValue);
      }
      if (this.connected) {
        this._subscribeWs(topic);
      }
    }

    setValue(topic, value) {
      if (!this.connected) return;

      if (!topic.pubuid) {
        this._publishWs(topic);
      }

      var encoded = encodeMsgpackEntry(topic.pubuid, value, topic.type);
      if (this.ws && this.ws.readyState === WebSocket.OPEN) {
        this.ws.send(encoded);
      }
    }

    close() {
      if (this.reconnectTimer) {
        clearTimeout(this.reconnectTimer);
        this.reconnectTimer = null;
      }
      if (this.ws) {
        this.ws.onclose = null;
        this.ws.close();
        this.ws = null;
      }
      this.connected = false;
    }
  }

  // ===== Minimal MessagePack Helpers =====

  function decodeMsgpackMessages(data) {
    var results = [];
    var view = new DataView(data.buffer, data.byteOffset, data.byteLength);
    var offset = 0;

    try {
      while (offset < data.length) {
        var r = decodeMsgpack(view, offset);
        offset = r.newOffset;
        if (Array.isArray(r.value)) {
          results.push(r.value);
        }
      }
    } catch (e) { /* return what we got */ }

    return results;
  }

  function decodeMsgpack(view, offset) {
    var byte = view.getUint8(offset);
    offset++;

    if ((byte & 0xf0) === 0x90) return decodeMsgpackArray(view, offset, byte & 0x0f);
    if (byte === 0xdc) { var l16 = view.getUint16(offset); return decodeMsgpackArray(view, offset + 2, l16); }
    if (byte === 0xdd) { var l32 = view.getUint32(offset); return decodeMsgpackArray(view, offset + 4, l32); }
    if ((byte & 0x80) === 0) return { value: byte, newOffset: offset };
    if ((byte & 0xe0) === 0xe0) return { value: byte - 256, newOffset: offset };
    if (byte === 0xcc) return { value: view.getUint8(offset), newOffset: offset + 1 };
    if (byte === 0xcd) return { value: view.getUint16(offset), newOffset: offset + 2 };
    if (byte === 0xce) return { value: view.getUint32(offset), newOffset: offset + 4 };
    if (byte === 0xcf) { var hi = view.getUint32(offset); var lo = view.getUint32(offset + 4); return { value: hi * 0x100000000 + lo, newOffset: offset + 8 }; }
    if (byte === 0xd0) return { value: view.getInt8(offset), newOffset: offset + 1 };
    if (byte === 0xd1) return { value: view.getInt16(offset), newOffset: offset + 2 };
    if (byte === 0xd2) return { value: view.getInt32(offset), newOffset: offset + 4 };
    if (byte === 0xd3) { var hi2 = view.getInt32(offset); var lo2 = view.getUint32(offset + 4); return { value: hi2 * 0x100000000 + lo2, newOffset: offset + 8 }; }
    if (byte === 0xca) return { value: view.getFloat32(offset), newOffset: offset + 4 };
    if (byte === 0xcb) return { value: view.getFloat64(offset), newOffset: offset + 8 };
    if ((byte & 0xe0) === 0xa0) return decodeMsgpackStr(view, offset, byte & 0x1f);
    if (byte === 0xd9) return decodeMsgpackStr(view, offset + 1, view.getUint8(offset));
    if (byte === 0xda) return decodeMsgpackStr(view, offset + 2, view.getUint16(offset));
    if (byte === 0xdb) return decodeMsgpackStr(view, offset + 4, view.getUint32(offset));
    if (byte === 0xc4) { var bl = view.getUint8(offset); return { value: new Uint8Array(view.buffer, view.byteOffset + offset + 1, bl), newOffset: offset + 1 + bl }; }
    if (byte === 0xc5) { var bl2 = view.getUint16(offset); return { value: new Uint8Array(view.buffer, view.byteOffset + offset + 2, bl2), newOffset: offset + 2 + bl2 }; }
    if (byte === 0xc0) return { value: null, newOffset: offset };
    if (byte === 0xc3) return { value: true, newOffset: offset };
    if (byte === 0xc2) return { value: false, newOffset: offset };
    if ((byte & 0xf0) === 0x80) return decodeMsgpackMap(view, offset, byte & 0x0f);
    if (byte === 0xde) { var ml = view.getUint16(offset); return decodeMsgpackMap(view, offset + 2, ml); }

    return { value: undefined, newOffset: offset };
  }

  function decodeMsgpackArray(view, offset, len) {
    var arr = [];
    for (var i = 0; i < len; i++) {
      var r = decodeMsgpack(view, offset);
      arr.push(r.value);
      offset = r.newOffset;
    }
    return { value: arr, newOffset: offset };
  }

  function decodeMsgpackStr(view, offset, len) {
    var bytes = new Uint8Array(view.buffer, view.byteOffset + offset, len);
    var str = new TextDecoder().decode(bytes);
    return { value: str, newOffset: offset + len };
  }

  function decodeMsgpackMap(view, offset, len) {
    var obj = {};
    for (var i = 0; i < len; i++) {
      var r1 = decodeMsgpack(view, offset);
      var r2 = decodeMsgpack(view, r1.newOffset);
      obj[r1.value] = r2.value;
      offset = r2.newOffset;
    }
    return { value: obj, newOffset: offset };
  }

  function encodeMsgpackEntry(pubuid, value, type) {
    var timestamp = Date.now() * 1000;
    var typeNum = getTypeNum(type);
    var parts = [];
    parts.push(0x94);
    encodeMsgpackInt(parts, pubuid);
    encodeMsgpackInt(parts, timestamp);
    encodeMsgpackInt(parts, typeNum);
    encodeMsgpackValue(parts, value, type);
    return new Uint8Array(parts);
  }

  function getTypeNum(type) {
    switch (type) {
      case 'boolean': return 0;
      case 'double': return 1;
      case 'int': return 2;
      case 'float': return 3;
      case 'string': return 4;
      case 'json': return 4;
      case 'string[]': return 20;
      default: return 4;
    }
  }

  function encodeMsgpackInt(parts, value) {
    if (value >= 0 && value < 128) {
      parts.push(value);
    } else if (value >= 0 && value < 256) {
      parts.push(0xcc, value);
    } else if (value >= 0 && value < 65536) {
      parts.push(0xcd, (value >> 8) & 0xff, value & 0xff);
    } else if (value >= 0 && value < 0x100000000) {
      parts.push(0xce, (value >> 24) & 0xff, (value >> 16) & 0xff, (value >> 8) & 0xff, value & 0xff);
    } else {
      var hi = Math.floor(value / 0x100000000);
      var lo = value >>> 0;
      parts.push(0xcf, (hi >> 24) & 0xff, (hi >> 16) & 0xff, (hi >> 8) & 0xff, hi & 0xff,
        (lo >> 24) & 0xff, (lo >> 16) & 0xff, (lo >> 8) & 0xff, lo & 0xff);
    }
  }

  function encodeMsgpackValue(parts, value, type) {
    if (type === 'string' || type === 'json') {
      var encoded = new TextEncoder().encode(value);
      if (encoded.length < 32) {
        parts.push(0xa0 | encoded.length);
      } else if (encoded.length < 256) {
        parts.push(0xd9, encoded.length);
      } else if (encoded.length < 65536) {
        parts.push(0xda, (encoded.length >> 8) & 0xff, encoded.length & 0xff);
      } else {
        parts.push(0xdb, (encoded.length >> 24) & 0xff, (encoded.length >> 16) & 0xff, (encoded.length >> 8) & 0xff, encoded.length & 0xff);
      }
      for (var i = 0; i < encoded.length; i++) parts.push(encoded[i]);
    } else if (type === 'double') {
      parts.push(0xcb);
      var buf = new ArrayBuffer(8);
      new DataView(buf).setFloat64(0, value);
      var bytes = new Uint8Array(buf);
      for (var j = 0; j < bytes.length; j++) parts.push(bytes[j]);
    } else if (type === 'boolean') {
      parts.push(value ? 0xc3 : 0xc2);
    } else if (type === 'int') {
      encodeMsgpackInt(parts, value);
    } else if (type === 'string[]') {
      var arr = Array.isArray(value) ? value : [value];
      if (arr.length < 16) {
        parts.push(0x90 | arr.length);
      } else {
        parts.push(0xdc, (arr.length >> 8) & 0xff, arr.length & 0xff);
      }
      for (var k = 0; k < arr.length; k++) {
        encodeMsgpackValue(parts, String(arr[k]), 'string');
      }
    } else {
      encodeMsgpackValue(parts, String(value), 'string');
    }
  }

  // Singleton factory
  var instances = new Map();

  return {
    NetworkTables: {
      getInstanceByURI: function (uri) {
        if (instances.has(uri)) {
          return instances.get(uri);
        }
        var client = new NT4Client(uri);
        instances.set(uri, client);
        return client;
      },
    },
  };
})();
