(function () {
    try {
        // Try to access localStorage to see if it works
        var x = '__storage_test__';
        localStorage.setItem(x, x);
        localStorage.removeItem(x);
    } catch (e) {
        // Storage is blocked (e.g. privacy settings, iframe), mock it to prevent crashes
        console.warn('localStorage access blocked. Mocking it to prevent application crash.');

        var data = {};
        var storageMock = {
            getItem: function (k) { return data[k] || null; },
            setItem: function (k, v) { data[k] = String(v); },
            removeItem: function (k) { delete data[k]; },
            clear: function () { data = {}; },
            key: function (i) { return Object.keys(data)[i] || null; },
            get length() { return Object.keys(data).length; }
        };

        try {
            Object.defineProperty(window, 'localStorage', {
                value: storageMock,
                writable: true,
                configurable: true
            });

            // Also mock sessionStorage just in case
            Object.defineProperty(window, 'sessionStorage', {
                value: storageMock, // Share same mock or create new one, doesn't matter much for crash prevention
                writable: true,
                configurable: true
            });
        } catch (defErr) {
            console.error('Failed to mock localStorage:', defErr);
        }
    }
})();
