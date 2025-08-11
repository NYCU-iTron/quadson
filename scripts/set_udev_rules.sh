VENDOR_ID="1d50"
PRODUCT_ID="606f"
RULE_FILE="/etc/udev/rules.d/99-canusb.rules"

echo "建立 udev 規則檔: $RULE_FILE"
echo "SUBSYSTEM==\"usb\", ATTR{idVendor}==\"$VENDOR_ID\", ATTR{idProduct}==\"$PRODUCT_ID\", MODE=\"0666\"" > "$RULE_FILE"

echo "重新載入 udev 規則..."
udevadm control --reload-rules
udevadm trigger

echo "已完成設定！"
echo "請拔掉 USB 裝置並重新插入，之後可不用 sudo 存取該設備。"