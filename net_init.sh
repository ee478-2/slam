sudo sysctl -w net.ipv4.ip_forward=1
sudo iptables -t nat -C POSTROUTING -o eth0 -j MASQUERADE 2>/dev/null || sudo iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
sudo iptables -I FORWARD 1 -i wlan0 -o eth0 -j ACCEPT
sudo iptables -I FORWARD 1 -i eth0 -o wlan0 -m state --state RELATED,ESTABLISHED -j ACCEPT
