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
        distanceMetres: 1.0,
        durationSeconds: 2.0,
        headingDegrees: 0,
        timeoutSeconds: 5.0,
      },
      fields: [
        { key: 'direction', label: 'Direction (deg)', type: 'number', min: 0, max: 360, step: 1, hint: 'Field-oriented direction of travel (blue alliance)' },
        { key: 'speedMPS', label: 'Speed (m/s)', type: 'number', min: 0, max: 5.36, step: 0.1, hint: 'Translation speed' },
        { key: 'mode', label: 'Mode', type: 'select', options: [{ value: 'distance', label: 'Distance (odometry)' }, { value: 'time', label: 'Time-based' }] },
        { key: 'distanceMetres', label: 'Distance (m)', type: 'number', min: 0, max: 20, step: 0.1, hint: 'Only used in distance mode', showIf: function (v) { return v.mode === 'distance'; } },
        { key: 'durationSeconds', label: 'Duration (s)', type: 'number', min: 0, max: 15, step: 0.1, hint: 'Only used in time mode', showIf: function (v) { return v.mode === 'time'; } },
        { key: 'headingDegrees', label: 'Heading (deg)', type: 'number', min: -180, max: 360, step: 1, hint: 'Robot facing direction (heading hold)' },
        { key: 'timeoutSeconds', label: 'Timeout (s)', type: 'number', min: 0, max: 15, step: 0.5, hint: 'Safety timeout for distance mode', showIf: function (v) { return v.mode === 'distance'; } },
      ],
      summarize: function (v) {
        if (v.mode === 'distance') {
          return v.direction + '\u00b0 @ ' + v.speedMPS + ' m/s, ' + v.distanceMetres + 'm, hdg ' + v.headingDegrees + '\u00b0';
        }
        return v.direction + '\u00b0 @ ' + v.speedMPS + ' m/s, ' + v.durationSeconds + 's, hdg ' + v.headingDegrees + '\u00b0';
      },
      validate: function (v) {
        var errors = [];
        if (v.speedMPS <= 0) errors.push('Speed must be > 0');
        if (v.mode === 'distance' && v.distanceMetres <= 0) errors.push('Distance must be > 0');
        if (v.mode === 'time' && v.durationSeconds <= 0) errors.push('Duration must be > 0');
        return errors;
      },
      serialize: function (v) {
        var out = { type: 'drive', direction: v.direction, speedMPS: v.speedMPS, mode: v.mode, headingDegrees: v.headingDegrees };
        if (v.mode === 'distance') { out.distanceMetres = v.distanceMetres; out.timeoutSeconds = v.timeoutSeconds; }
        else { out.durationSeconds = v.durationSeconds; }
        return out;
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

    parallel: {
      label: 'Parallel',
      icon: 'P',
      colorClass: 'parallel',
      defaultValues: { type: 'parallel', endCondition: 'all', commands: [] },
      fields: [
        { key: 'endCondition', label: 'End Condition', type: 'select', options: [
          { value: 'all', label: 'All finish (ParallelCommandGroup)' },
          { value: 'first', label: 'First finishes (ParallelRaceGroup)' },
        ]},
      ],
      summarize: function (v) {
        var n = (v.commands || []).length;
        return (v.endCondition === 'all' ? 'All' : 'Race') + ', ' + n + ' cmd' + (n !== 1 ? 's' : '');
      },
      validate: function (v) {
        var e = [];
        if (!v.commands || v.commands.length < 2) e.push('Parallel group needs at least 2 commands');
        return e;
      },
      serialize: function (v) {
        return {
          type: 'parallel',
          endCondition: v.endCondition,
          commands: (v.commands || []).map(function (child) {
            var childDef = COMMAND_TYPES[child.type];
            return childDef ? childDef.serialize(child) : child;
          }),
        };
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
      input.addEventListener('change', function () { onChange(input.value); });
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

  function deserializeStep(jsonStep) {
    var typeDef = COMMAND_TYPES[jsonStep.type];
    if (!typeDef) return jsonStep;
    var merged = Object.assign({}, typeDef.defaultValues, jsonStep);
    if (jsonStep.type === 'parallel' && Array.isArray(jsonStep.commands)) {
      merged.commands = jsonStep.commands.map(deserializeStep);
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
  };
})();
