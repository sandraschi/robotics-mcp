/**
 * Python environment checker
 * Runs after npm install to validate Python setup
 */

import { spawn } from 'child_process';
import { fileURLToPath } from 'url';
import { dirname } from 'path';

const pythonCmd = process.platform === 'win32' ? 'python' : 'python3';
const moduleName = 'robotics_mcp';

// Check if Python is available
const checkPython = () => {
    return new Promise((resolve) => {
        const proc = spawn(pythonCmd, ['--version'], {
            stdio: 'pipe'
        });
        
        proc.on('close', (code) => {
            resolve(code === 0);
        });
        
        proc.on('error', () => {
            resolve(false);
        });
    });
};

// Check if module is importable
const checkModule = () => {
    return new Promise((resolve) => {
        const moduleToImport = moduleName.split('.')[0];
        const proc = spawn(pythonCmd, ['-c', 'import ' + moduleToImport], {
            stdio: 'pipe'
        });
        
        proc.on('close', (code) => {
            resolve(code === 0);
        });
        
        proc.on('error', () => {
            resolve(false);
        });
    });
};

(async () => {
    const hasPython = await checkPython();
    if (!hasPython) {
        console.warn('[WARNING] Python not found. Install Python 3.10+ to use this package.');
        console.warn('   Command: ' + pythonCmd);
        process.exit(0); // Don't fail install, just warn
    }
    
    const hasModule = await checkModule();
    if (!hasModule) {
        console.warn('[WARNING] Python module "' + moduleName + '" not found.');
        console.warn('   Install dependencies with: pip install -e .');
        process.exit(0); // Don't fail install, just warn
    }
    
    console.log('[OK] Python environment check passed');
})();
