const dgram = require('dgram');
const mavlink = require('./mavlink_1.0');


class MavTransfer {

    constructor(io) {
        this.startMav();
        this.setupUdp();
    }

    setupUdp() {
        if (!this.mav) {
            console.log('wait mav init');
            setTimeout(this.startMav, 3000);
            return;
        }

        // address:port
        this.addresses = new Array();
        const self = this;

        this.client = dgram.createSocket('udp4');
        this.server = dgram.createSocket('udp4');

        this.server.on('connect', () => {
            console.log('on connected');
        });

        this.server.on('error', (err) => {
            console.log(`server error:\n${err.stack}`);
            server.close();
            setTimeout(this.setupUdp, 5000);
        });

        this.server.on('close', function () {
            console.log('server close');
            setTimeout(this.setupUdp, 5000);
        });

        this.server.on('message', (msg, rinfo) => {
            let recvHost = rinfo.address + ":" + rinfo.port;
            if (!self.addresses.includes(recvHost)) {
                self.addresses.push(recvHost);
            }
            this.mav.parseBuffer(msg);
        });

        this.server.on('listening', () => {
            const address = this.server.address();
            console.log(`server listening ${address.address}:${address.port}`);
        });

        this.server.bind(ConfigStore.getInstance().getStore().vehicle_udp_port);
    }

    startMav() {
        this.mav = new MAVLink({
            log: function (err, msg) {
                // console.log(err);
                // console.log(msg);
            }
        }, 255, 1);
        var self = this;
        this.mav.on("message", function (msg) {
            // VehicleStateに入力
            if (self.vehicleState) {
                self.vehicleState.handleMavlink(msg);
            }
        });
    }

    requestHomePosition() {
        if (!this.srcSystem) {
            return;
        }
        const sethome = new mavlink.messages.command_long(this.srcSystem, 0, mavlink.MAV_CMD_GET_HOME_POSITION, 0, 0, 0, 0, 0, 0, 0, 0);
        const buf = new Buffer(sethome.pack(this.mav));
        this.sendAll(buf);
    }

    sendAll(buf, cb) {
        _.forEach(this.addresses, (hostKey) => {
            let items = hostKey.split(':');
            this._send(buf, items[0], parseInt(items[1]), cb);
        });
    }

    _send(buf, host, port, cb) {
        if (this.server) {
            this.server.send(buf, 0, buf.length, port, host, function (err, bytes) {
                if (cb) {
                    cb(err, bytes);
                }
            });
        }
    }
}

module.exports = MavTransfer;
