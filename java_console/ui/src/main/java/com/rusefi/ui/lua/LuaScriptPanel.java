package com.rusefi.ui.lua;

import com.opensr5.ConfigurationImage;
import com.opensr5.ini.IniFileModel;
import com.opensr5.ini.IniFileReader;
import com.opensr5.ini.RawIniFile;
import com.opensr5.ini.field.IniField;
import com.rusefi.binaryprotocol.BinaryProtocol;
import com.rusefi.config.generated.Fields;
import com.rusefi.io.ConnectionStatusLogic;
import com.rusefi.io.LinkManager;
import com.rusefi.ui.MessagesPanel;
import com.rusefi.ui.UIContext;
import com.rusefi.core.preferences.storage.Node;
import com.rusefi.ui.util.URLLabel;
import com.rusefi.ui.widgets.AnyCommand;
import neoe.formatter.lua.LuaFormatter;

import javax.swing.*;
import java.awt.*;
import java.awt.event.ActionListener;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.charset.StandardCharsets;
import java.util.function.Consumer;

import static com.rusefi.ui.util.UiUtils.trueLayout;

public class LuaScriptPanel {
    private static final String LUA_SCRIPT_FIELD_NAME = "luaScript";

    private final UIContext context;
    private final JPanel mainPanel = new JPanel(new BorderLayout());
    private final AnyCommand command;
    private final TextEditor scriptText = new TextEditor();

    public LuaScriptPanel(UIContext context, Node config) {
        this.context = context;
        command = AnyCommand.createField(context, config, true, true);

        // Upper panel: command entry, etc
        JPanel upperPanel = new JPanel(new FlowLayout(FlowLayout.LEFT, 5, 0));

        JButton writeButton = new JButton("Write to ECU");
        JButton resetButton = new JButton("Reset/Reload Lua");
        JButton formatButton = new JButton("Format");
        JButton burnButton = new JButton("Burn to ECU");

        MessagesPanel mp = new MessagesPanel(null, config);

        mp.getMessagesView().listener = message -> {
            if (message.contains("BEEP"))
                Toolkit.getDefaultToolkit().beep();
        };

        writeButton.addActionListener(e -> {
            write();
            // resume messages on 'write new script to ECU'
            mp.setPaused(false);
        });
        resetButton.addActionListener(e -> resetLua());

        formatButton.addActionListener(e -> {
            String sourceCode = scriptText.getText();
            try {
                String formatted = new LuaFormatter().format(sourceCode, new LuaFormatter.Env());
                scriptText.setText(formatted);
            } catch (Exception ignored) {
                // todo: fix luaformatter no reason for exception
            }
        });

        burnButton.addActionListener(e -> {
            LinkManager linkManager = context.getLinkManager();

            linkManager.submit(() -> {
                BinaryProtocol bp = linkManager.getCurrentStreamState();
                bp.burn();
            });
        });

        upperPanel.add(formatButton);
        upperPanel.add(writeButton);
        upperPanel.add(resetButton);
        upperPanel.add(burnButton);
        upperPanel.add(command.getContent());
        upperPanel.add(new URLLabel("Lua Wiki", "https://github.com/rusefi/rusefi/wiki/Lua-Scripting"));

        // Center panel - script editor and log
        JPanel scriptPanel = new JPanel(new BorderLayout());
        scriptPanel.add(scriptText.getControl(), BorderLayout.CENTER);

        //centerPanel.add(, BorderLayout.WEST);
        JPanel messagesPanel = new JPanel(new BorderLayout());
        messagesPanel.add(BorderLayout.NORTH, mp.getButtonPanel());
        messagesPanel.add(BorderLayout.CENTER, mp.getMessagesScroll());

        ConnectionStatusLogic.INSTANCE.addListener(isConnected -> SwingUtilities.invokeLater(() -> {
            try {
                readFromECU();
            } catch (Throwable e) {
                System.out.println(e);
            }
        }));

        JSplitPane centerPanel = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, scriptPanel, messagesPanel);

        mainPanel.add(upperPanel, BorderLayout.NORTH);
        mainPanel.add(centerPanel, BorderLayout.CENTER);

        trueLayout(mainPanel);
        SwingUtilities.invokeLater(() -> centerPanel.setDividerLocation(centerPanel.getSize().width / 2));
    }

    public JPanel getPanel() {
        return mainPanel;
    }

    public ActionListener getTabSelectedListener() {
        return e -> {
            if (command != null)
                command.requestFocus();
        };
    }

    void readFromECU() {
        BinaryProtocol bp = context.getLinkManager().getCurrentStreamState();

        if (bp == null) {
            scriptText.setText("No ECU located");
            return;
        }

        ConfigurationImage image = bp.getControllerConfiguration();
        if (image == null) {
            scriptText.setText("No configuration image");
            return;
        }

        IniField luaScriptField = resolveLuaScriptField(bp, image);
        if (luaScriptField == null)
            return;

        ByteBuffer luaScriptBuffer = image.getByteBuffer(luaScriptField.getOffset(), luaScriptField.getSize());

        byte[] scriptArr = new byte[luaScriptField.getSize()];
        luaScriptBuffer.get(scriptArr);

        int i = findNullTerminator(scriptArr);
        scriptText.setText(new String(scriptArr, 0, i, StandardCharsets.US_ASCII));
    }

    private IniField resolveLuaScriptField(BinaryProtocol bp, ConfigurationImage image) {
        try {
            File iniFile = IniFileModel.getInstance().getSourceFile();
            if (iniFile == null) {
                setEditorMessage("Lua scripting unavailable: active INI is not loaded");
                return null;
            }

            RawIniFile ini;
            try (FileInputStream input = new FileInputStream(iniFile)) {
                ini = IniFileReader.read(input);
            }

            return resolveLuaScriptField(ini, bp.signature, image.getSize(), this::setEditorMessage);
        } catch (IOException | RuntimeException e) {
            setEditorMessage("Lua scripting unavailable: could not read active ECU schema");
        }
        return null;
    }

    static IniField resolveLuaScriptField(RawIniFile ini, String connectedSignature, int configImageSize) {
        return resolveLuaScriptField(ini, connectedSignature, configImageSize, message -> {
        });
    }

    private static IniField resolveLuaScriptField(RawIniFile ini, String connectedSignature, int configImageSize,
                                                  Consumer<String> onError) {
        try {
            if (connectedSignature == null || connectedSignature.isEmpty()) {
                onError.accept("Lua scripting unavailable: ECU signature is not available");
                return null;
            }

            String activeSignature = new IniFileModel().readIniFile(ini).getSignature();
            if (activeSignature == null || activeSignature.isEmpty() || !connectedSignature.equals(activeSignature)) {
                onError.accept("Lua scripting unavailable: active INI does not match ECU signature");
                return null;
            }

            for (RawIniFile.Line line : ini.getLines()) {
                String[] tokens = line.getTokens();
                if (tokens.length < 5 || !LUA_SCRIPT_FIELD_NAME.equalsIgnoreCase(tokens[0])
                        || !IniFileModel.FIELD_TYPE_STRING.equalsIgnoreCase(tokens[1])
                        || !"ASCII".equalsIgnoreCase(tokens[2]))
                    continue;

                int offset = Integer.parseInt(tokens[3]);
                int size = Integer.parseInt(tokens[4]);
                if (offset < 0 || size <= 0 || (long) offset + size > configImageSize) {
                    onError.accept("Lua scripting unavailable: active INI has an invalid luaScript field");
                    return null;
                }
                return new ActiveLuaScriptField(offset, size);
            }

            onError.accept("Lua scripting unavailable for this ECU profile: active INI has no luaScript field");
        } catch (RuntimeException e) {
            onError.accept("Lua scripting unavailable: could not read active ECU schema");
        }
        return null;
    }

    @SuppressWarnings("StatementWithEmptyBody")
    private static int findNullTerminator(byte[] scriptArr) {
        int i;
        for (i = 0; i < scriptArr.length && scriptArr[i] != 0; i++) ;
        return i;
    }

    void write() {
        String script = scriptText.getText();

        LinkManager linkManager = context.getLinkManager();

        linkManager.submit(() -> {
            BinaryProtocol bp = linkManager.getCurrentStreamState();
            if (bp == null) {
                setEditorMessage("No ECU located");
                return;
            }

            ConfigurationImage image = bp.getControllerConfiguration();
            if (image == null) {
                setEditorMessage("No configuration image");
                return;
            }

            IniField luaScriptField = resolveLuaScriptField(bp, image);
            if (luaScriptField == null)
                return;

            byte[] scriptBytes = script.getBytes(StandardCharsets.US_ASCII);
            if (scriptBytes.length >= luaScriptField.getSize()) {
                setEditorMessage("Lua script is too large for this ECU profile: "
                        + scriptBytes.length + "/" + (luaScriptField.getSize() - 1) + " bytes");
                return;
            }

            byte[] paddedScript = new byte[luaScriptField.getSize()];
            System.arraycopy(scriptBytes, 0, paddedScript, 0, scriptBytes.length);

            int idx = 0;
            int remaining;

            do {
                remaining = paddedScript.length - idx;
                int thisWrite = Math.min(remaining, Fields.BLOCKING_FACTOR);

                bp.writeData(paddedScript, idx, luaScriptField.getOffset() + idx, thisWrite);

                idx += thisWrite;

                remaining -= thisWrite;
            } while (remaining > 0);

// need a way to modify script on the fly with shorter execution gaps to keep E65 CAN network happy
// todo: auto-burn on console close check box in case of Lua changes?
// todo: check box for auto-burn?
//            bp.burn();

            // Burning doesn't reload lua script, so we have to do it manually
            resetLua();
        });
    }

    private void setEditorMessage(String message) {
        if (SwingUtilities.isEventDispatchThread()) {
            scriptText.setText(message);
        } else {
            SwingUtilities.invokeLater(() -> scriptText.setText(message));
        }
    }

    void resetLua() {
        this.context.getCommandQueue().write("luareset");
    }

    private static final class ActiveLuaScriptField extends IniField {
        private final int size;

        private ActiveLuaScriptField(int offset, int size) {
            super(LUA_SCRIPT_FIELD_NAME, offset);
            this.size = size;
        }

        @Override
        public int getSize() {
            return size;
        }
    }
}
