/**
 * Command type definitions, property form generation, and validation rules
 * for the FRC 1310 Auto Configuration Dashboard.
 *
 * All headings/directions are blue-alliance oriented.
 * The robot handles red alliance offset at runtime.
 */

window.CommandPalette = (function () {
  'use strict';

  var _stepIdCounter = 0;

  function generateStepId() {
    return 'step-' + (++_stepIdCounter);
  }

  var COMMAND_TYPES = {
    drive: {
      label: 'Drive',
      icon: 'D',
      colorClass: 'drive',
      defaultValues: {
        type: 'drive',
        direction: 0,
        speedMPS: 1.0,
        mode: 'distance',
        frame: 'field',
        vxMPS: 1.0,
        vyMPS: 0.0,
        distanceMetres: 1.0,
        durationSeconds: 2.0,
        headingDegrees: 0,
        timeoutSeconds: 5.0,
        xMetres: 1.0,
        yMetres: 1.0,
        positionToleranceMetres: 0.05,
        headingToleranceDegrees: 2.0,
      },
      fields: [
        { key: 'mode', label: 'Mode', type: 'select', options: [{ value: 'distance', label: 'Distance (odometry)' }, { value: 'time', label: 'Time-based' }, { value: 'to_pose', label: 'To Pose (odometry)' }, { value: 'velocity', label: 'Velocity (vx/vy + duration)' }] },
        { key: 'direction', label: 'Direction (deg)', type: 'number', min: 0, max: 360, step: 1, hint: 'Field-oriented direction of travel (blue alliance)', showIf: function (v) { return v.mode === 'distance' || v.mode === 'time'; } },
        { key: 'speedMPS', label: 'Speed (m/s)', type: 'number', min: 0, max: 5.36, step: 0.1, hint: 'Translation speed / max approach speed', showIf: function (v) { return v.mode !== 'velocity'; } },
        { key: 'frame', label: 'Frame', type: 'select', options: [{ value: 'field', label: 'Field Oriented' }, { value: 'robot', label: 'Robot Oriented' }], showIf: function (v) { return v.mode === 'velocity'; } },
        { key: 'vxMPS', label: 'Vx (m/s)', type: 'number', min: -5.36, max: 5.36, step: 0.1, hint: 'Forward velocity in selected frame', showIf: function (v) { return v.mode === 'velocity'; } },
        { key: 'vyMPS', label: 'Vy (m/s)', type: 'number', min: -5.36, max: 5.36, step: 0.1, hint: 'Left/right velocity in selected frame', showIf: function (v) { return v.mode === 'velocity'; } },
        { key: 'distanceMetres', label: 'Distance (m)', type: 'number', min: 0, max: 20, step: 0.1, hint: 'Only used in distance mode', showIf: function (v) { return v.mode === 'distance'; } },
        { key: 'durationSeconds', label: 'Duration (s)', type: 'number', min: 0, max: 15, step: 0.1, hint: 'Used in time and velocity modes', showIf: function (v) { return v.mode === 'time' || v.mode === 'velocity'; } },
        { key: 'xMetres', label: 'Target X (m)', type: 'number', min: -20, max: 20, step: 0.1, hint: 'Only used in to-pose mode (blue alliance field)', showIf: function (v) { return v.mode === 'to_pose'; } },
        { key: 'yMetres', label: 'Target Y (m)', type: 'number', min: -20, max: 20, step: 0.1, hint: 'Only used in to-pose mode (blue alliance field)', showIf: function (v) { return v.mode === 'to_pose'; } },
        { key: 'headingDegrees', label: 'Heading (deg)', type: 'number', min: -180, max: 360, step: 1, hint: 'Robot facing direction (heading hold / target heading)' },
        { key: 'positionToleranceMetres', label: 'Position Tol (m)', type: 'number', min: 0.01, max: 2.0, step: 0.01, showIf: function (v) { return v.mode === 'to_pose'; } },
        { key: 'headingToleranceDegrees', label: 'Heading Tol (deg)', type: 'number', min: 0.5, max: 20, step: 0.5, showIf: function (v) { return v.mode === 'to_pose'; } },
        { key: 'timeoutSeconds', label: 'Timeout (s)', type: 'number', min: 0, max: 15, step: 0.5, hint: 'Safety timeout', showIf: function (v) { return v.mode === 'distance' || v.mode === 'to_pose'; } },
      ],
      summarize: function (v) {
        if (v.mode === 'distance') {
          return v.direction + '\u00b0 @ ' + v.speedMPS + ' m/s, ' + v.distanceMetres + 'm, hdg ' + v.headingDegrees + '\u00b0';
        }
        if (v.mode === 'to_pose') {
          return 'to (' + v.xMetres + ', ' + v.yMetres + '), ' + v.speedMPS + ' m/s max, hdg ' + v.headingDegrees + '\u00b0';
        }
        if (v.mode === 'velocity') {
          return v.frame + ' vx ' + v.vxMPS + ', vy ' + v.vyMPS + ', ' + v.durationSeconds + 's, hdg ' + v.headingDegrees + '\u00b0';
        }
        return v.direction + '\u00b0 @ ' + v.speedMPS + ' m/s, ' + v.durationSeconds + 's, hdg ' + v.headingDegrees + '\u00b0';
      },
      validate: function (v) {
        var errors = [];
        if (v.mode !== 'velocity' && v.speedMPS <= 0) errors.push('Speed must be > 0');
        if (v.mode === 'distance' && v.distanceMetres <= 0) errors.push('Distance must be > 0');
        if (v.mode === 'time' && v.durationSeconds <= 0) errors.push('Duration must be > 0');
        if (v.mode === 'velocity' && v.durationSeconds <= 0) errors.push('Duration must be > 0');
        if (v.mode === 'to_pose') {
          if (v.positionToleranceMetres <= 0) errors.push('Position tolerance must be > 0');
          if (v.headingToleranceDegrees <= 0) errors.push('Heading tolerance must be > 0');
          if (v.timeoutSeconds <= 0) errors.push('Timeout must be > 0');
        }
        if (['distance', 'time', 'to_pose', 'velocity'].indexOf(v.mode) === -1) {
          errors.push('Mode must be distance, time, to_pose, or velocity');
        }
        return errors;
      },
      serialize: function (v) {
        var out = { type: 'drive', mode: v.mode, headingDegrees: v.headingDegrees };
        if (v.mode === 'distance') {
          out.speedMPS = v.speedMPS;
          out.direction = v.direction;
          out.distanceMetres = v.distanceMetres;
          out.timeoutSeconds = v.timeoutSeconds;
        } else if (v.mode === 'time') {
          out.speedMPS = v.speedMPS;
          out.direction = v.direction;
          out.durationSeconds = v.durationSeconds;
        } else if (v.mode === 'velocity') {
          out.frame = v.frame;
          out.vxMPS = v.vxMPS;
          out.vyMPS = v.vyMPS;
          out.durationSeconds = v.durationSeconds;
        } else {
          out.speedMPS = v.speedMPS;
          out.xMetres = v.xMetres;
          out.yMetres = v.yMetres;
          out.positionToleranceMetres = v.positionToleranceMetres;
          out.headingToleranceDegrees = v.headingToleranceDegrees;
          out.timeoutSeconds = v.timeoutSeconds;
        }
        return out;
      },
    },

    set_pose: {
      label: 'Set Pose',
      icon: 'O',
      colorClass: 'set_pose',
      defaultValues: {
        type: 'set_pose',
        xMetres: 0.0,
        yMetres: 0.0,
        headingDegrees: 0,
      },
      fields: [
        { key: 'xMetres', label: 'Pose X (m)', type: 'number', min: -20, max: 20, step: 0.1, hint: 'Blue-alliance field coordinate' },
        { key: 'yMetres', label: 'Pose Y (m)', type: 'number', min: -20, max: 20, step: 0.1, hint: 'Blue-alliance field coordinate' },
        { key: 'headingDegrees', label: 'Heading (deg)', type: 'number', min: -180, max: 360, step: 1, hint: 'Blue-alliance heading' },
      ],
      summarize: function (v) {
        return '(' + v.xMetres + ', ' + v.yMetres + '), hdg ' + v.headingDegrees + '\u00b0';
      },
      validate: function () {
        return [];
      },
      serialize: function (v) {
        return {
          type: 'set_pose',
          xMetres: v.xMetres,
          yMetres: v.yMetres,
          headingDegrees: v.headingDegrees,
        };
      },
    },

    rotate: {
      label: 'Rotate',
      icon: 'R',
      colorClass: 'rotate',
      defaultValues: { type: 'rotate', headingDegrees: 90, timeoutSeconds: 3.0 },
      fields: [
        { key: 'headingDegrees', label: 'Heading (deg)', type: 'number', min: -180, max: 360, step: 1, hint: 'Target field-oriented heading (blue alliance)' },
        { key: 'timeoutSeconds', label: 'Timeout (s)', type: 'number', min: 0, max: 10, step: 0.5 },
      ],
      summarize: function (v) { return 'to ' + v.headingDegrees + '\u00b0, timeout ' + v.timeoutSeconds + 's'; },
      validate: function (v) { var e = []; if (v.timeoutSeconds <= 0) e.push('Timeout must be > 0'); return e; },
      serialize: function (v) { return { type: 'rotate', headingDegrees: v.headingDegrees, timeoutSeconds: v.timeoutSeconds }; },
    },

    shooter: {
      label: 'Shooter',
      icon: 'S',
      colorClass: 'shooter',
      defaultValues: { type: 'shooter', action: 'on', rpm: 3000, hoodPosition: 0.0, kickerSpeed: -0.7, kickerDelaySeconds: 2.0, durationSeconds: 4.0 },
      fields: [
        { key: 'action', label: 'Action', type: 'select', options: [
          { value: 'on', label: 'On (until interrupted)' },
          { value: 'on_with_duration', label: 'On with duration' },
          { value: 'off', label: 'Off' },
        ]},
        { key: 'rpm', label: 'RPM', type: 'number', min: 0, max: 6200, step: 100, hint: 'Shooter speed', showIf: function (v) { return v.action !== 'off'; } },
        { key: 'hoodPosition', label: 'Hood Position', type: 'number', min: 0, max: 1, step: 0.05, hint: '0.0 = flat, 1.0 = full angle', showIf: function (v) { return v.action !== 'off'; } },
        { key: 'kickerSpeed', label: 'Kicker Speed', type: 'number', min: -1, max: 1, step: 0.1, hint: 'Negative = feed into shooter', showIf: function (v) { return v.action !== 'off'; } },
        { key: 'kickerDelaySeconds', label: 'Kicker Delay (s)', type: 'number', min: 0, max: 10, step: 0.5, hint: 'Delay before kicker engages', showIf: function (v) { return v.action !== 'off'; } },
        { key: 'durationSeconds', label: 'Duration (s)', type: 'number', min: 0, max: 15, step: 0.5, showIf: function (v) { return v.action === 'on_with_duration'; } },
      ],
      summarize: function (v) {
        if (v.action === 'off') return 'OFF';
        if (v.action === 'on_with_duration') return v.rpm + ' RPM, ' + v.durationSeconds + 's, hood ' + v.hoodPosition;
        return v.rpm + ' RPM, hood ' + v.hoodPosition;
      },
      validate: function (v) {
        var e = [];
        if (v.action !== 'off') {
          if (v.rpm <= 0) e.push('RPM must be > 0');
          if (v.action === 'on_with_duration' && v.durationSeconds <= 0) e.push('Duration must be > 0');
        }
        return e;
      },
      serialize: function (v) {
        if (v.action === 'off') return { type: 'shooter', action: 'off' };
        var out = { type: 'shooter', action: v.action, rpm: v.rpm, hoodPosition: v.hoodPosition, kickerSpeed: v.kickerSpeed, kickerDelaySeconds: v.kickerDelaySeconds };
        if (v.action === 'on_with_duration') out.durationSeconds = v.durationSeconds;
        return out;
      },
    },

    intake: {
      label: 'Intake',
      icon: 'I',
      colorClass: 'intake',
      defaultValues: { type: 'intake', action: 'on', speed: 0.8, durationSeconds: 2.0 },
      fields: [
        { key: 'action', label: 'Action', type: 'select', options: [
          { value: 'on', label: 'On (until interrupted)' },
          { value: 'on_with_duration', label: 'On with duration' },
          { value: 'off', label: 'Off' },
        ]},
        { key: 'speed', label: 'Speed', type: 'number', min: -1, max: 1, step: 0.1, hint: 'Motor speed (-1 to 1)', showIf: function (v) { return v.action !== 'off'; } },
        { key: 'durationSeconds', label: 'Duration (s)', type: 'number', min: 0, max: 15, step: 0.5, showIf: function (v) { return v.action === 'on_with_duration'; } },
      ],
      summarize: function (v) {
        if (v.action === 'off') return 'OFF';
        if (v.action === 'on_with_duration') return 'speed ' + v.speed + ', ' + v.durationSeconds + 's';
        return 'speed ' + v.speed;
      },
      validate: function (v) {
        var e = [];
        if (v.action !== 'off' && v.speed === 0) e.push('Speed should not be 0');
        if (v.action === 'on_with_duration' && v.durationSeconds <= 0) e.push('Duration must be > 0');
        return e;
      },
      serialize: function (v) {
        if (v.action === 'off') return { type: 'intake', action: 'off' };
        var out = { type: 'intake', action: v.action, speed: v.speed };
        if (v.action === 'on_with_duration') out.durationSeconds = v.durationSeconds;
        return out;
      },
    },

    delay: {
      label: 'Delay',
      icon: 'W',
      colorClass: 'delay',
      defaultValues: { type: 'delay', durationSeconds: 1.0 },
      fields: [
        { key: 'durationSeconds', label: 'Duration (s)', type: 'number', min: 0, max: 15, step: 0.1 },
      ],
      summarize: function (v) { return v.durationSeconds + 's'; },
      validate: function (v) { var e = []; if (v.durationSeconds <= 0) e.push('Duration must be > 0'); return e; },
      serialize: function (v) { return { type: 'delay', durationSeconds: v.durationSeconds }; },
    },

    hold: {
      label: 'Hold Drive',
      icon: 'H',
      colorClass: 'hold',
      defaultValues: { type: 'hold', durationSeconds: 1.0 },
      fields: [
        { key: 'durationSeconds', label: 'Duration (s)', type: 'number', min: 0, max: 15, step: 0.1, hint: '0 = hold until interrupted (useful with parallel deadline)' },
      ],
      summarize: function (v) {
        if (!v.durationSeconds || v.durationSeconds <= 0) return 'until interrupted';
        return v.durationSeconds + 's';
      },
      validate: function (v) {
        var e = [];
        if (v.durationSeconds < 0) e.push('Duration must be >= 0');
        return e;
      },
      serialize: function (v) {
        return { type: 'hold', durationSeconds: v.durationSeconds };
      },
    },

    face_target: {
      label: 'Face Target',
      icon: 'F',
      colorClass: 'face_target',
      defaultValues: {
        type: 'face_target',
        target: 'hub',
        targetXMetres: 2.0,
        targetYMetres: 2.0,
        headingToleranceDegrees: 3.0,
        timeoutSeconds: 3.0,
      },
      fields: [
        { key: 'target', label: 'Target', type: 'select', options: [{ value: 'hub', label: 'Hub' }, { value: 'point', label: 'Field Point' }] },
        { key: 'targetXMetres', label: 'Target X (m)', type: 'number', min: -20, max: 20, step: 0.1, showIf: function (v) { return v.target === 'point'; } },
        { key: 'targetYMetres', label: 'Target Y (m)', type: 'number', min: -20, max: 20, step: 0.1, showIf: function (v) { return v.target === 'point'; } },
        { key: 'headingToleranceDegrees', label: 'Heading Tol (deg)', type: 'number', min: 0.5, max: 20, step: 0.5 },
        { key: 'timeoutSeconds', label: 'Timeout (s)', type: 'number', min: 0.1, max: 15, step: 0.5 },
      ],
      summarize: function (v) {
        if (v.target === 'point') return 'point (' + v.targetXMetres + ', ' + v.targetYMetres + ')';
        return 'hub';
      },
      validate: function (v) {
        var e = [];
        if (v.timeoutSeconds <= 0) e.push('Timeout must be > 0');
        if (v.headingToleranceDegrees <= 0) e.push('Heading tolerance must be > 0');
        return e;
      },
      serialize: function (v) {
        return {
          type: 'face_target',
          target: v.target,
          targetXMetres: v.targetXMetres,
          targetYMetres: v.targetYMetres,
          headingToleranceDegrees: v.headingToleranceDegrees,
          timeoutSeconds: v.timeoutSeconds,
        };
      },
    },

    vision_approach_tag: {
      label: 'Vision Approach',
      icon: 'T',
      colorClass: 'vision_approach_tag',
      defaultValues: {
        type: 'vision_approach_tag',
        rightSide: false,
        timeoutSeconds: 5.0,
      },
      fields: [
        {
          key: 'rightSide',
          label: 'Tower Side',
          type: 'select',
          coerce: 'boolean',
          options: [
            { value: 'false', label: 'Left Side' },
            { value: 'true', label: 'Right Side' },
          ],
        },
        { key: 'timeoutSeconds', label: 'Timeout (s)', type: 'number', min: 0.1, max: 15, step: 0.5 },
      ],
      summarize: function (v) {
        return (v.rightSide ? 'right' : 'left') + ', timeout ' + v.timeoutSeconds + 's';
      },
      validate: function (v) {
        var e = [];
        if (v.timeoutSeconds <= 0) e.push('Timeout must be > 0');
        return e;
      },
      serialize: function (v) {
        return {
          type: 'vision_approach_tag',
          rightSide: !!v.rightSide,
          timeoutSeconds: v.timeoutSeconds,
        };
      },
    },

    parallel: {
      label: 'Parallel',
      icon: 'P',
      colorClass: 'parallel',
      defaultValues: { type: 'parallel', endCondition: 'all', deadlineIndex: 1, timeoutSeconds: 0, commands: [] },
      fields: [
        { key: 'endCondition', label: 'End Condition', type: 'select', options: [
          { value: 'all', label: 'All finish (ParallelCommandGroup)' },
          { value: 'first', label: 'First finishes (ParallelRaceGroup)' },
          { value: 'deadline', label: 'Deadline child finishes (ParallelDeadlineGroup)' },
        ]},
        { key: 'deadlineIndex', label: 'Deadline Child #', type: 'number', min: 1, max: 10, step: 1, showIf: function (v) { return v.endCondition === 'deadline'; } },
        { key: 'timeoutSeconds', label: 'Timeout (s)', type: 'number', min: 0, max: 15, step: 0.5, hint: '0 = no timeout' },
      ],
      summarize: function (v) {
        var n = (v.commands || []).length;
        var mode = v.endCondition === 'all' ? 'All' : (v.endCondition === 'first' ? 'Race' : ('Deadline #' + (v.deadlineIndex || 1)));
        var summary = mode + ', ' + n + ' cmd' + (n !== 1 ? 's' : '');
        if (v.timeoutSeconds > 0) summary += ', timeout ' + v.timeoutSeconds + 's';
        return summary;
      },
      validate: function (v) {
        var e = [];
        if (!v.commands || v.commands.length < 2) e.push('Parallel group needs at least 2 commands');
        if (v.endCondition === 'deadline') {
          var maxCmd = v.commands ? v.commands.length : 0;
          if (v.deadlineIndex < 1 || v.deadlineIndex > maxCmd) {
            e.push('Deadline child # must be between 1 and ' + maxCmd);
          }
        }
        return e;
      },
      serialize: function (v) {
        var out = {
          type: 'parallel',
          endCondition: v.endCondition,
          commands: (v.commands || []).map(function (child) {
            var childDef = COMMAND_TYPES[child.type];
            return childDef ? childDef.serialize(child) : child;
          }),
        };
        if (v.endCondition === 'deadline') {
          out.deadlineIndex = Math.max(0, Math.floor((v.deadlineIndex || 1) - 1));
        }
        if (v.timeoutSeconds > 0) {
          out.timeoutSeconds = v.timeoutSeconds;
        }
        return out;
      },
    },

    sequential: {
      label: 'Sequential',
      icon: 'Q',
      colorClass: 'sequential',
      defaultValues: { type: 'sequential', timeoutSeconds: 0, commands: [] },
      fields: [
        { key: 'timeoutSeconds', label: 'Timeout (s)', type: 'number', min: 0, max: 15, step: 0.5, hint: '0 = no timeout' },
      ],
      summarize: function (v) {
        var n = (v.commands || []).length;
        var summary = n + ' cmd' + (n !== 1 ? 's' : '');
        if (v.timeoutSeconds > 0) summary += ', timeout ' + v.timeoutSeconds + 's';
        return summary;
      },
      validate: function (v) {
        var e = [];
        if (!v.commands || v.commands.length < 2) e.push('Sequential group needs at least 2 commands');
        return e;
      },
      serialize: function (v) {
        var out = {
          type: 'sequential',
          commands: (v.commands || []).map(function (child) {
            var childDef = COMMAND_TYPES[child.type];
            return childDef ? childDef.serialize(child) : child;
          }),
        };
        if (v.timeoutSeconds > 0) {
          out.timeoutSeconds = v.timeoutSeconds;
        }
        return out;
      },
    },
  };

  function createPaletteItems(container) {
    container.innerHTML = '';
    var keys = Object.keys(COMMAND_TYPES);
    for (var i = 0; i < keys.length; i++) {
      var typeKey = keys[i];
      var def = COMMAND_TYPES[typeKey];
      var item = document.createElement('div');
      item.className = 'palette-item';
      item.setAttribute('draggable', 'true');
      item.dataset.commandType = typeKey;
      item.innerHTML = '<span class="palette-icon ' + def.colorClass + '">' + def.icon + '</span><span>' + def.label + '</span>';
      container.appendChild(item);
    }
  }

  function renderPropertiesForm(container, stepData, onChange, onDelete) {
    container.innerHTML = '';
    if (!stepData) {
      container.innerHTML = '<div class="properties-empty">Select a command in the sequence to edit its properties.</div>';
      return;
    }
    var typeDef = COMMAND_TYPES[stepData.type];
    if (!typeDef) {
      container.innerHTML = '<div class="properties-empty">Unknown command type.</div>';
      return;
    }

    var form = document.createElement('div');
    form.className = 'properties-form';

    var header = document.createElement('div');
    header.style.cssText = 'display:flex; align-items:center; justify-content:space-between; margin-bottom:4px;';
    header.innerHTML = '<span class="step-type-badge ' + typeDef.colorClass + '" style="font-size:13px; padding:3px 10px;">' + typeDef.label + '</span>';
    var deleteBtn = document.createElement('button');
    deleteBtn.className = 'btn btn-danger';
    deleteBtn.style.fontSize = '11px';
    deleteBtn.style.padding = '3px 10px';
    deleteBtn.textContent = 'Delete';
    deleteBtn.addEventListener('click', onDelete);
    header.appendChild(deleteBtn);
    form.appendChild(header);

    form.appendChild(createDivider());

    for (var i = 0; i < typeDef.fields.length; i++) {
      var field = typeDef.fields[i];
      if (field.showIf && !field.showIf(stepData)) continue;
      form.appendChild(createFormField(field, stepData[field.key], function (key) {
        return function (value) { onChange(key, value); };
      }(field.key)));
    }

    var errors = typeDef.validate(stepData);
    if (errors.length > 0) {
      form.appendChild(createDivider());
      for (var j = 0; j < errors.length; j++) {
        var errEl = document.createElement('div');
        errEl.className = 'field-error';
        errEl.style.padding = '2px 0';
        errEl.textContent = errors[j];
        form.appendChild(errEl);
      }
    }

    container.appendChild(form);
  }

  function createFormField(field, value, onChange) {
    var group = document.createElement('div');
    group.className = 'form-group';
    var label = document.createElement('label');
    label.textContent = field.label;
    group.appendChild(label);

    var input;
    if (field.type === 'select') {
      input = document.createElement('select');
      for (var i = 0; i < field.options.length; i++) {
        var option = document.createElement('option');
        option.value = field.options[i].value;
        option.textContent = field.options[i].label;
        if (field.options[i].value === String(value)) option.selected = true;
        input.appendChild(option);
      }
      input.value = value;
      input.addEventListener('change', function () {
        if (field.coerce === 'boolean') {
          onChange(input.value === 'true');
        } else if (field.coerce === 'number') {
          onChange(parseFloat(input.value) || 0);
        } else {
          onChange(input.value);
        }
      });
    } else if (field.type === 'number') {
      input = document.createElement('input');
      input.type = 'number';
      if (field.min !== undefined) input.min = field.min;
      if (field.max !== undefined) input.max = field.max;
      if (field.step !== undefined) input.step = field.step;
      input.value = value != null ? value : '';
      input.addEventListener('change', function () { onChange(parseFloat(input.value) || 0); });
      input.addEventListener('keydown', function (e) { if (e.key === 'Enter') e.target.blur(); });
    } else {
      input = document.createElement('input');
      input.type = 'text';
      input.value = value != null ? value : '';
      input.addEventListener('input', function () { onChange(input.value); });
    }

    group.appendChild(input);
    if (field.hint) {
      var hint = document.createElement('div');
      hint.className = 'field-hint';
      hint.textContent = field.hint;
      group.appendChild(hint);
    }
    return group;
  }

  function createDivider() {
    var hr = document.createElement('hr');
    hr.className = 'form-divider';
    return hr;
  }

  /**
   * Migrate legacy step format to new type names.
   * {type:"drive", mode:"distance"} -> {type:"drive_distance"}
   * {type:"face_target", target:"hub"} -> {type:"face_hub"}
   * {type:"face_target", target:"point"} -> {type:"face_field_point"}
   * {type:"drive_velocity"} -> {type:"drive_velocity"} (already correct)
   */
  function migrateStep(jsonStep) {
    if (!jsonStep || !jsonStep.type) return jsonStep;
    var step = Object.assign({}, jsonStep);

    if (step.type === 'drive' && step.mode) {
      if (step.mode === 'distance') step.type = 'drive_distance';
      else if (step.mode === 'time') step.type = 'drive_timed';
      else if (step.mode === 'velocity') step.type = 'drive_velocity';
      else if (step.mode === 'to_pose') step.type = 'drive_to_pose';
      delete step.mode;
    }
    if (step.type === 'face_target') {
      if (step.target === 'hub') step.type = 'face_hub';
      else if (step.target === 'point') step.type = 'face_field_point';
      delete step.target;
    }

    // Recursively migrate parallel/sequential children
    if ((step.type === 'parallel' || step.type === 'sequential') && Array.isArray(step.commands)) {
      step.commands = step.commands.map(migrateStep);
    }
    return step;
  }

  /**
   * Update COMMAND_TYPES from robot-published metadata.
   * Structural types (parallel, delay) are preserved as hardcoded.
   * Dynamic types are built from the metadata array.
   */
  function updateFromRobotMetadata(metadata) {
    if (!Array.isArray(metadata)) return;

    // Cache in localStorage for offline use
    try {
      localStorage.setItem('1310-command-metadata', JSON.stringify(metadata));
    } catch (e) { /* ignore */ }

    _applyMetadata(metadata);
  }

  function _loadCachedMetadata() {
    try {
      var cached = localStorage.getItem('1310-command-metadata');
      if (cached) {
        var metadata = JSON.parse(cached);
        _applyMetadata(metadata);
      }
    } catch (e) { /* ignore */ }
  }

  var CATEGORY_COLORS = {
    drive: 'drive',
    shooter: 'shooter',
    intake: 'intake',
    utility: 'delay',
  };

  var CATEGORY_ICONS = {
    drive: 'D',
    shooter: 'S',
    intake: 'I',
    utility: 'U',
  };

  function _applyMetadata(metadata) {
    // Keep structural types
    var preserved = {};
    if (COMMAND_TYPES.parallel) preserved.parallel = COMMAND_TYPES.parallel;
    if (COMMAND_TYPES.sequential) preserved.sequential = COMMAND_TYPES.sequential;
    if (COMMAND_TYPES.delay) preserved.delay = COMMAND_TYPES.delay;

    // Clear all non-structural types
    var keys = Object.keys(COMMAND_TYPES);
    for (var i = 0; i < keys.length; i++) {
      if (keys[i] !== 'parallel' && keys[i] !== 'sequential' && keys[i] !== 'delay') {
        delete COMMAND_TYPES[keys[i]];
      }
    }

    // Build dynamic types from metadata
    for (var j = 0; j < metadata.length; j++) {
      var meta = metadata[j];
      COMMAND_TYPES[meta.type] = _buildDynamicCommandType(meta);
    }

    // Re-add structural types at the end
    if (preserved.delay) COMMAND_TYPES.delay = preserved.delay;
    if (preserved.parallel) COMMAND_TYPES.parallel = preserved.parallel;
    if (preserved.sequential) COMMAND_TYPES.sequential = preserved.sequential;
  }

  function _buildDynamicCommandType(meta) {
    var defaults = { type: meta.type, timeoutSeconds: 0 };
    var fields = [];

    var params = meta.params || [];
    for (var i = 0; i < params.length; i++) {
      var p = params[i];
      var field = { key: p.name, label: _paramLabel(p), hint: p.description || '' };

      if (p.options && p.options.length > 0) {
        field.type = 'select';
        field.options = p.options.map(function (opt) { return { value: opt, label: opt }; });
        defaults[p.name] = p.options[0];
      } else if (p.javaType === 'boolean') {
        field.type = 'select';
        field.coerce = 'boolean';
        field.options = [{ value: 'false', label: 'No' }, { value: 'true', label: 'Yes' }];
        defaults[p.name] = p.defaultValue !== 0;
      } else {
        field.type = 'number';
        if (p.min !== undefined) field.min = p.min;
        if (p.max !== undefined) field.max = p.max;
        field.step = _guessStep(p);
        defaults[p.name] = p.defaultValue || 0;
      }

      fields.push(field);
    }

    fields.push({ key: 'timeoutSeconds', label: 'Timeout (s)', type: 'number', min: 0, max: 15, step: 0.5, hint: '0 = no timeout' });

    var category = meta.category || 'utility';

    return {
      label: _typeLabel(meta.type),
      icon: CATEGORY_ICONS[category] || meta.type.charAt(0).toUpperCase(),
      colorClass: CATEGORY_COLORS[category] || 'delay',
      description: meta.description || '',
      defaultValues: defaults,
      fields: fields,
      summarize: function (v) {
        var parts = [];
        for (var k = 0; k < params.length; k++) {
          var key = params[k].name;
          if (v[key] !== undefined && v[key] !== null) {
            var unit = params[k].unit ? params[k].unit : '';
            parts.push(key + '=' + v[key] + unit);
          }
          if (parts.length >= 3) break;
        }
        if (v.timeoutSeconds > 0) parts.push('timeout=' + v.timeoutSeconds + 's');
        return parts.join(', ') || meta.type;
      },
      validate: function () { return []; },
      serialize: function (v) {
        var out = { type: meta.type };
        for (var k = 0; k < params.length; k++) {
          var key = params[k].name;
          if (v[key] !== undefined) out[key] = v[key];
        }
        if (v.timeoutSeconds > 0) out.timeoutSeconds = v.timeoutSeconds;
        return out;
      },
    };
  }

  function _typeLabel(type) {
    return type.replace(/_/g, ' ').replace(/\b\w/g, function (c) { return c.toUpperCase(); });
  }

  function _paramLabel(p) {
    var label = p.name.replace(/([A-Z])/g, ' $1').replace(/_/g, ' ');
    label = label.charAt(0).toUpperCase() + label.slice(1);
    if (p.unit) label += ' (' + p.unit + ')';
    return label;
  }

  function _guessStep(p) {
    if (p.unit === 'deg') return 1;
    if (p.unit === 's') return 0.1;
    if (p.unit === 'm') return 0.1;
    if (p.unit === 'm/s') return 0.1;
    if (p.unit === 'rpm') return 100;
    if (p.max !== undefined && p.min !== undefined) {
      var range = p.max - p.min;
      if (range <= 2) return 0.05;
      if (range <= 20) return 0.5;
      return 1;
    }
    return 0.1;
  }

  // Load cached metadata on startup
  _loadCachedMetadata();

  function deserializeStep(jsonStep) {
    // Migrate legacy format
    jsonStep = migrateStep(jsonStep);
    var typeDef = COMMAND_TYPES[jsonStep.type];
    if (!typeDef) return jsonStep;
    var merged = Object.assign({}, typeDef.defaultValues, jsonStep);
    if ((jsonStep.type === 'parallel' || jsonStep.type === 'sequential') && Array.isArray(jsonStep.commands)) {
      merged.commands = jsonStep.commands.map(deserializeStep);
    }
    // Convert 0-based JSON deadlineIndex to 1-based UI value
    if (jsonStep.type === 'parallel' && jsonStep.deadlineIndex !== undefined) {
      merged.deadlineIndex = (jsonStep.deadlineIndex || 0) + 1;
    }
    return merged;
  }

  function serializeStep(step) {
    var typeDef = COMMAND_TYPES[step.type];
    if (!typeDef) return step;
    return typeDef.serialize(step);
  }

  function getStepSummary(step) {
    var typeDef = COMMAND_TYPES[step.type];
    if (!typeDef) return JSON.stringify(step);
    return typeDef.summarize(step);
  }

  function validateStep(step) {
    var typeDef = COMMAND_TYPES[step.type];
    if (!typeDef) return ['Unknown command type: ' + step.type];
    return typeDef.validate(step);
  }

  return {
    COMMAND_TYPES: COMMAND_TYPES,
    createPaletteItems: createPaletteItems,
    renderPropertiesForm: renderPropertiesForm,
    deserializeStep: deserializeStep,
    serializeStep: serializeStep,
    getStepSummary: getStepSummary,
    validateStep: validateStep,
    generateStepId: generateStepId,
    updateFromRobotMetadata: updateFromRobotMetadata,
    migrateStep: migrateStep,
  };
})();
