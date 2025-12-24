#!/usr/bin/env node
/**
 * NPM wrapper for robotics-mcp MCP Server
 * 
 * This script launches the Python MCP server via FastMCP.
 * Python and dependencies must be installed separately.
 */

import { spawn } from 'child_process';
import { fileURLToPath } from 'url';
import { dirname, join } from 'path';
import { existsSync } from 'fs';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
const serverRoot = join(__dirname, '..');

// Determine Python command based on platform
const pythonCmd = process.platform === 'win32' ? 'python' : 'python3';

// Try to find Python module entry point
// Common patterns: python -m module_name or python -m module_name.__main__
const moduleName = 'robotics_mcp';
const possibleCommands = [
    ['-m', moduleName],
    ['-m', moduleName + '.__main__'],
    ['-m', moduleName + '.server'],
];

// Function to test if a Python command works
function testPythonCommand(args) {
    return new Promise((resolve) => {
        const testProc = spawn(pythonCmd, ['-c', import ], {
            stdio: 'pipe',
            cwd: serverRoot
        });
        
        testProc.on('close', (code) => {
            resolve(code === 0);
        });
        
        testProc.on('error', () => {
            resolve(false);
        });
    });
}

// Find working command
async function findWorkingCommand() {
    for (const args of possibleCommands) {
        const works = await testPythonCommand(args);
        if (works) {
            return args;
        }
    }
    // Fallback to first option
    return possibleCommands[0];
}

// Main execution
(async () => {
    try {
        const commandArgs = await findWorkingCommand();
        
        const proc = spawn(pythonCmd, commandArgs, {
            stdio: 'inherit',
            cwd: serverRoot,
            env: {
                ...process.env,
                PYTHONUNBUFFERED: '1'
            }
        });
        
        proc.on('error', (err) => {
            console.error('Error starting Python server: ' + err.message);
            console.error('\nMake sure Python is installed and ''robotics_mcp'' is available.');
            console.error('Try: pip install -e .');
            process.exit(1);
        });
        
        proc.on('exit', (code) => {
            process.exit(code || 0);
        });
        
        // Handle signals
        process.on('SIGINT', () => {
            proc.kill('SIGINT');
        });
        
        process.on('SIGTERM', () => {
            proc.kill('SIGTERM');
        });
        
    } catch (error) {
        console.error('Failed to start server: ' + error.message);
        process.exit(1);
    }
})();
